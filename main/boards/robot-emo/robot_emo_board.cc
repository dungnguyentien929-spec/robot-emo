#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "led/single_led.h"
#include "mcp_server.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <new>
#include <string>
#include <utility>
#include <wifi_manager.h>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "RobotEmoBoard"

namespace {

constexpr gpio_num_t kMotorPins[] = {
    GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14
};
constexpr uint8_t kMax30102Address = 0x57;
constexpr uint8_t kMax30102PartId = 0x15;
constexpr uint32_t kFingerThreshold = 50000;
constexpr uint64_t kHeartRateStaleUs = 2500000;
constexpr uint64_t kDiagnosticWindowUs = 15000000;

class MotorDriver {
private:
    SemaphoreHandle_t mutex_ = nullptr;
    esp_timer_handle_t timer_ = nullptr;
    bool ready_ = false;
    bool moving_ = false;
    int64_t deadline_us_ = 0;

    static void AutoStop(void* arg) {
        static_cast<MotorDriver*>(arg)->StopFromTimer();
    }

    void SetAllLow() {
        for (gpio_num_t pin : kMotorPins) {
            esp_err_t err = gpio_set_level(pin, 0);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set motor GPIO%d LOW: %s", pin, esp_err_to_name(err));
            }
        }
    }

    esp_err_t SetOutputs(bool in1, bool in2, bool in3, bool in4) {
        const uint32_t levels[] = {in1, in2, in3, in4};
        for (size_t i = 0; i < 4; ++i) {
            esp_err_t err = gpio_set_level(kMotorPins[i], levels[i]);
            if (err != ESP_OK) {
                SetAllLow();
                return err;
            }
        }
        return ESP_OK;
    }

    void StopFromTimer() {
        if (mutex_ == nullptr || xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
            SetAllLow();
            return;
        }

        if (moving_ && esp_timer_get_time() >= deadline_us_) {
            SetAllLow();
            moving_ = false;
            deadline_us_ = 0;
            ESP_LOGI(TAG, "Motor auto-stop: all four outputs LOW");
        }
        xSemaphoreGive(mutex_);
    }

public:
    MotorDriver() {
        gpio_config_t config = {};
        config.pin_bit_mask = (1ULL << GPIO_NUM_11) | (1ULL << GPIO_NUM_12) |
                              (1ULL << GPIO_NUM_13) | (1ULL << GPIO_NUM_14);
        config.mode = GPIO_MODE_OUTPUT;
        config.pull_up_en = GPIO_PULLUP_DISABLE;
        config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        config.intr_type = GPIO_INTR_DISABLE;
        esp_err_t err = gpio_config(&config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure motor GPIOs: %s", esp_err_to_name(err));
            return;
        }

        SetAllLow();

        mutex_ = xSemaphoreCreateMutex();
        if (mutex_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create motor mutex; motors remain LOW");
            return;
        }

        esp_timer_create_args_t timer_args = {
            .callback = AutoStop,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "robot_motor_stop",
            .skip_unhandled_events = true,
        };
        err = esp_timer_create(&timer_args, &timer_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create motor auto-stop timer: %s", esp_err_to_name(err));
            return;
        }
        ready_ = true;
        ESP_LOGI(TAG, "Motor outputs initialized LOW (GPIO11-14)");
    }

    esp_err_t Move(const std::string& direction, int duration_ms) {
        if (!ready_) {
            return ESP_ERR_INVALID_STATE;
        }
        if (duration_ms < 500 || duration_ms > 5000) {
            return ESP_ERR_INVALID_ARG;
        }

        bool levels[4];
        if (direction == "forward") {
            levels[0] = true;  levels[1] = false; levels[2] = true;  levels[3] = false;
        } else if (direction == "backward") {
            levels[0] = false; levels[1] = true;  levels[2] = false; levels[3] = true;
        } else if (direction == "left") {
            levels[0] = false; levels[1] = true;  levels[2] = true;  levels[3] = false;
        } else if (direction == "right") {
            levels[0] = true;  levels[1] = false; levels[2] = false; levels[3] = true;
        } else {
            return ESP_ERR_INVALID_ARG;
        }

        if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }

        esp_timer_stop(timer_);
        SetAllLow();
        esp_err_t err = SetOutputs(levels[0], levels[1], levels[2], levels[3]);
        if (err == ESP_OK) {
            deadline_us_ = esp_timer_get_time() + static_cast<int64_t>(duration_ms) * 1000;
            err = esp_timer_start_once(timer_, static_cast<uint64_t>(duration_ms) * 1000);
        }
        if (err == ESP_OK) {
            moving_ = true;
        } else {
            SetAllLow();
            moving_ = false;
            deadline_us_ = 0;
        }
        xSemaphoreGive(mutex_);
        return err;
    }

    esp_err_t Stop() {
        if (mutex_ == nullptr) {
            SetAllLow();
            return ESP_ERR_INVALID_STATE;
        }
        if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
            SetAllLow();
            return ESP_ERR_TIMEOUT;
        }
        if (timer_ != nullptr) {
            esp_timer_stop(timer_);
        }
        SetAllLow();
        moving_ = false;
        deadline_us_ = 0;
        xSemaphoreGive(mutex_);
        return ESP_OK;
    }
};

class HeartRateSensor {
private:
    static constexpr uint8_t kRegFifoWritePtr = 0x04;
    static constexpr uint8_t kRegFifoReadPtr = 0x06;
    static constexpr uint8_t kRegFifoData = 0x07;

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t device_ = nullptr;
    TaskHandle_t task_ = nullptr;
    SemaphoreHandle_t mutex_ = nullptr;
    std::string init_error_ = "MAX30102 is not initialized";

    int32_t ir_baseline_ = 0;
    int32_t previous_ac_ = 0;
    uint32_t beat_intervals_ms_[4] = {};
    size_t interval_count_ = 0;
    size_t interval_next_ = 0;
    uint64_t last_beat_us_ = 0;
    int latest_bpm_ = 0;

    // Raw IR/baseline diagnostics, logged once per second for a window after each tool call.
    uint64_t diag_until_us_ = 0;
    uint64_t last_diag_log_us_ = 0;

    bool ReadRegister(uint8_t reg, uint8_t* data, size_t length) {
        return i2c_master_transmit_receive(device_, &reg, 1, data, length, 100) == ESP_OK;
    }

    bool WriteRegister(uint8_t reg, uint8_t value) {
        uint8_t buffer[] = {reg, value};
        return i2c_master_transmit(device_, buffer, sizeof(buffer), 100) == ESP_OK;
    }

    static void SensorTask(void* arg) {
        static_cast<HeartRateSensor*>(arg)->Run();
    }

    void UpdateBeat(uint32_t ir, uint64_t now_us) {
        int32_t log_baseline = 0;
        bool log_sample = false;
        xSemaphoreTake(mutex_, portMAX_DELAY);

        if (ir <= kFingerThreshold) {
            ir_baseline_ = 0;
            previous_ac_ = 0;
            interval_count_ = 0;
            interval_next_ = 0;
            last_beat_us_ = 0;
            latest_bpm_ = 0;
        } else {
            if (ir_baseline_ == 0) {
                ir_baseline_ = static_cast<int32_t>(ir);
            }
            ir_baseline_ += (static_cast<int32_t>(ir) - ir_baseline_) / 32;
            const int32_t ac = static_cast<int32_t>(ir) - ir_baseline_;
            const int32_t threshold = ir_baseline_ / 200;
            const int32_t peak_threshold = threshold > 200 ? threshold : 200;

            if (previous_ac_ <= peak_threshold && ac > peak_threshold &&
                (last_beat_us_ == 0 || now_us - last_beat_us_ >= 300000)) {
                if (last_beat_us_ != 0) {
                    uint64_t interval_us = now_us - last_beat_us_;
                    if (interval_us >= 300000 && interval_us <= 2000000) {
                        beat_intervals_ms_[interval_next_] = static_cast<uint32_t>(interval_us / 1000);
                        interval_next_ = (interval_next_ + 1) % 4;
                        if (interval_count_ < 4) {
                            ++interval_count_;
                        }
                        if (interval_count_ == 4) {
                            uint64_t total_ms = 0;
                            for (uint32_t interval : beat_intervals_ms_) {
                                total_ms += interval;
                            }
                            latest_bpm_ = static_cast<int>((4ULL * 60000 + total_ms / 2) / total_ms);
                        }
                    } else {
                        interval_count_ = 0;
                        interval_next_ = 0;
                    }
                }
                last_beat_us_ = now_us;
            }
            previous_ac_ = ac;

            if (last_beat_us_ != 0 && now_us - last_beat_us_ > kHeartRateStaleUs) {
                interval_count_ = 0;
                interval_next_ = 0;
                last_beat_us_ = 0;
                latest_bpm_ = 0;
            }
        }

        if (now_us < diag_until_us_ && now_us - last_diag_log_us_ >= 1000000) {
            last_diag_log_us_ = now_us;
            log_baseline = ir_baseline_;
            log_sample = true;
        }

        xSemaphoreGive(mutex_);
        if (log_sample) {
            ESP_LOGI(TAG, "MAX30102 diag sample: IR=%lu baseline=%ld finger_threshold=%lu",
                     static_cast<unsigned long>(ir), static_cast<long>(log_baseline),
                     static_cast<unsigned long>(kFingerThreshold));
        }
    }

    void Run() {
        while (true) {
            uint8_t write_ptr = 0;
            uint8_t read_ptr = 0;
            if (ReadRegister(kRegFifoWritePtr, &write_ptr, 1) &&
                ReadRegister(kRegFifoReadPtr, &read_ptr, 1) &&
                ((write_ptr - read_ptr) & 0x1F) != 0) {
                uint8_t sample[6];
                if (ReadRegister(kRegFifoData, sample, sizeof(sample))) {
                    uint32_t ir = ((static_cast<uint32_t>(sample[3]) << 16) |
                                   (static_cast<uint32_t>(sample[4]) << 8) | sample[5]) &
                                  0x3FFFF;
                    UpdateBeat(ir, esp_timer_get_time());
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

public:
    HeartRateSensor() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_1,
            .sda_io_num = GPIO_NUM_8,
            .scl_io_num = GPIO_NUM_9,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = 1},
        };
        esp_err_t err = i2c_new_master_bus(&bus_config, &bus_);
        if (err != ESP_OK) {
            init_error_ = "Failed to initialize MAX30102 I2C bus 2: " +
                          std::string(esp_err_to_name(err));
            ESP_LOGE(TAG, "%s", init_error_.c_str());
            return;
        }

        err = i2c_master_probe(bus_, kMax30102Address, 100);
        if (err != ESP_OK) {
            init_error_ = "MAX30102 not found at I2C address 0x57";
            ESP_LOGE(TAG, "%s", init_error_.c_str());
            i2c_del_master_bus(bus_);
            bus_ = nullptr;
            return;
        }

        i2c_device_config_t device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = kMax30102Address,
            .scl_speed_hz = 400000,
            .scl_wait_us = 0,
            .flags = {.disable_ack_check = 0},
        };
        err = i2c_master_bus_add_device(bus_, &device_config, &device_);
        uint8_t part_id = 0;
        if (err != ESP_OK || !ReadRegister(0xFF, &part_id, 1) || part_id != kMax30102PartId) {
            init_error_ = "MAX30102 at 0x57 did not respond with the expected part ID";
            ESP_LOGE(TAG, "%s", init_error_.c_str());
            return;
        }

        if (!WriteRegister(0x09, 0x40)) {
            init_error_ = "Failed to reset MAX30102";
            ESP_LOGE(TAG, "%s", init_error_.c_str());
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!WriteRegister(0x08, 0x4F) || !WriteRegister(0x09, 0x03) ||
            !WriteRegister(0x0A, 0x27) || !WriteRegister(0x0C, 0x24) ||
            !WriteRegister(0x0D, 0x24) || !WriteRegister(kRegFifoWritePtr, 0) ||
            !WriteRegister(0x05, 0) || !WriteRegister(kRegFifoReadPtr, 0)) {
            init_error_ = "Failed to configure MAX30102";
            ESP_LOGE(TAG, "%s", init_error_.c_str());
            return;
        }

        mutex_ = xSemaphoreCreateMutex();
        if (mutex_ == nullptr) {
            init_error_ = "Failed to create MAX30102 state mutex";
            ESP_LOGE(TAG, "%s", init_error_.c_str());
            return;
        }
        if (xTaskCreate(SensorTask, "heart_rate", 4096, this, 1, &task_) != pdPASS) {
            task_ = nullptr;
            init_error_ = "Failed to create MAX30102 sensor task";
            ESP_LOGE(TAG, "%s", init_error_.c_str());
            return;
        }
        init_error_.clear();
        ESP_LOGI(TAG, "MAX30102 found at 0x57; sensor task stack=4096 bytes, priority=1");
    }

    // Non-blocking: runs on the main event loop, so it only reads the sampler task's state.
    ToolResult GetHeartRate() {
        if (task_ == nullptr) {
            return std::unexpected(init_error_);
        }

        const uint64_t now_us = esp_timer_get_time();
        xSemaphoreTake(mutex_, portMAX_DELAY);
        const bool fresh = latest_bpm_ > 0 && last_beat_us_ != 0 &&
                           now_us - last_beat_us_ <= kHeartRateStaleUs;
        const int bpm = latest_bpm_;
        const size_t beats = interval_count_;
        diag_until_us_ = now_us + kDiagnosticWindowUs;
        last_diag_log_us_ = 0;  // log the next sample, then once per second
        xSemaphoreGive(mutex_);

        if (fresh) {
            return "BPM: " + std::to_string(bpm) + ". This is a reference value, not medical.";
        }
        return "No fresh reading yet. Ask the user to rest a finger on the sensor, keep still, "
               "and ask again in about 5 seconds. Beats detected: " +
               std::to_string(beats) + "/4. This is a reference value, not medical.";
    }
};

}  // namespace

class RobotEmoBoard : public WifiBoard {
private:
    MotorDriver motor_driver_;
    HeartRateSensor heart_rate_sensor_;
    i2c_master_bus_handle_t display_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeSsd1306Display() {
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .scl_speed_hz = 400 * 1000,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(display_i2c_bus_, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install SSD1306 driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

#ifdef SH1106
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
#endif

        ESP_LOGI(TAG, "SSD1306 driver installed");
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize display");
            display_ = new (std::nothrow) NoDisplay();
            if (display_ == nullptr) {
                ESP_LOGE(TAG, "Failed to allocate fallback display");
            }
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = new (std::nothrow) OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        if (display_ == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate OLED display");
        }
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool(
            "self.robot.move",
            "Use when the user asks the robot to move forward, backward, or turn left/right. "
            "Direction must be exactly one of: forward, backward, left, right. "
            "Starts the selected movement and automatically stops after duration_ms (500-5000; default 3000).",
            PropertyList({Property("direction", kPropertyTypeString),
                          Property("duration_ms", kPropertyTypeInteger, 3000, 500, 5000)}),
            [this](const PropertyList& properties) -> ToolResult {
                const std::string direction = properties["direction"].value<std::string>();
                const int duration_ms = properties["duration_ms"].value<int>();
                if (!WifiManager::GetInstance().IsConnected()) {
                    ESP_LOGI(TAG,
                             "MCP self.robot.move args={direction:%s,duration_ms:%d} result=error: "
                             "network disconnected",
                             direction.c_str(), duration_ms);
                    return std::unexpected("Cannot move: Wi-Fi connection is not available");
                }

                esp_err_t err = motor_driver_.Move(direction, duration_ms);
                if (err != ESP_OK) {
                    const std::string result = std::string("error: ") + esp_err_to_name(err);
                    ESP_LOGI(TAG,
                             "MCP self.robot.move args={direction:%s,duration_ms:%d} result=%s",
                             direction.c_str(), duration_ms, result.c_str());
                    return std::unexpected(std::string("Motor move failed: ") + esp_err_to_name(err));
                }
                const std::string result = "moving " + direction + " for " +
                                           std::to_string(duration_ms) + " ms";
                ESP_LOGI(TAG,
                         "MCP self.robot.move args={direction:%s,duration_ms:%d} result=%s",
                         direction.c_str(), duration_ms, result.c_str());
                return result;
            });

        mcp_server.AddTool(
            "self.robot.stop",
            "Use when the user asks the robot to stop moving. Immediately sets all four motor inputs LOW.",
            PropertyList(),
            [this](const PropertyList&) -> ToolResult {
                esp_err_t err = motor_driver_.Stop();
                if (err != ESP_OK) {
                    ESP_LOGI(TAG, "MCP self.robot.stop args={} result=error: %s",
                             esp_err_to_name(err));
                    return std::unexpected(std::string("Motor stop failed: ") + esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "MCP self.robot.stop args={} result=all four motor inputs LOW");
                return std::string("Stopped; all four motor inputs are LOW.");
            });

        mcp_server.AddTool(
            "self.sensor.get_heart_rate",
            "Use when the user asks the robot to measure heart rate. Returns immediately with the "
            "latest fresh MAX30102 reading (BPM). If there is no fresh reading yet, it says so; "
            "then ask the user to rest a finger on the sensor, keep still, and ask again in about "
            "5 seconds. This is a reference value, not medical.",
            PropertyList(),
            [this](const PropertyList&) -> ToolResult {
                ToolResult result = heart_rate_sensor_.GetHeartRate();
                if (!result) {
                    ESP_LOGI(TAG, "MCP self.sensor.get_heart_rate args={} result=error: %s",
                             result.error().c_str());
                    return result;
                }
                const std::string& text = std::get<std::string>(*result);
                ESP_LOGI(TAG, "MCP self.sensor.get_heart_rate args={} result=%s", text.c_str());
                return result;
            });
    }

public:
    RobotEmoBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        RegisterMcpTools();
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
    }

    virtual void SetNetworkEventCallback(NetworkEventCallback callback) override {
        WifiBoard::SetNetworkEventCallback(
            [this, callback = std::move(callback)](NetworkEvent event, const std::string& data) {
                if (event == NetworkEvent::Disconnected) {
                    motor_driver_.Stop();
                    ESP_LOGW(TAG, "Network/server connection lost; safety-stopped motors");
                }
                if (callback) {
                    callback(event, data);
                }
            });
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(RobotEmoBoard);

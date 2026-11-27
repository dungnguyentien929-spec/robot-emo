#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "weather_report.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/idf_additions.h>
#include <algorithm>
#include <atomic>
#include <cstring>
#include <new>
#include <string>
#include <utility>
#include <wifi_manager.h>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "RobotEmoBoard"

namespace {

// Motor direction inversion (firmware fix for mirrored motor mounting, no rewiring).
// true swaps that motor's two L298N input levels for every direction, so its wheel turns
// the other way. Flip one if that wheel spins backwards on "forward".
constexpr bool kInvertMotorA = false;  // Motor A: IN1/IN2 = GPIO11/GPIO12
constexpr bool kInvertMotorB = true;   // Motor B: IN3/IN4 = GPIO13/GPIO14

constexpr gpio_num_t kMotorPins[] = {
    GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14
};
constexpr uint8_t kMax30102Address = 0x57;
constexpr uint8_t kMax30102PartId = 0x15;
// MAX30102 SPO2_CONFIG (0x0A): ADC range 16384 nA (bits 6:5 = 11), 100 samples/s, 411 us
// pulses (18-bit). At the 4096 nA range (0x27) a firmly pressed finger saturated IR at 262143.
constexpr uint8_t kSpo2Config = 0x67;
// Finger threshold on raw IR. At 4096 nA, IR was ~800 with no finger, 5000-40000 while the
// finger was being placed and saturated when pressed; at 16384 nA expect a quarter of that.
constexpr uint32_t kFingerThreshold = 8000;
constexpr uint64_t kHeartRateStaleUs = 2500000;
// MAX30102 LED1 (red) and LED2 (IR) pulse amplitude, registers 0x0C/0x0D: 0.2 mA/LSB, 0x3C = 12 mA.
constexpr uint8_t kLedCurrent = 0x3C;

// Weather place for self.weather.get. To change it, edit kWeatherPlace and, in kWeatherUrl,
// latitude, longitude and (outside Vietnam) timezone, then rebuild.
constexpr const char* kWeatherPlace = "Thu Duc City, Ho Chi Minh City, Vietnam";
constexpr const char* kWeatherUrl =
    "https://api.open-meteo.com/v1/forecast?latitude=10.85&longitude=106.77"
    "&current=temperature_2m,relative_humidity_2m,apparent_temperature,precipitation,"
    "weather_code,wind_speed_10m"
    "&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max"
    "&timezone=Asia%2FHo_Chi_Minh&forecast_days=2";

// Speaker volume cap. On a weak supply, the MAX98357A's current peaks at high volume brown
// out the ESP32 (seen in a log while the robot spoke). Volume scales amplitude by
// (volume/100)^2, so 50 draws about half the peak current of the default 70.
constexpr int kMaxOutputVolume = 50;

class RobotEmoAudioCodec : public NoAudioCodecSimplex {
public:
    using NoAudioCodecSimplex::NoAudioCodecSimplex;

    void Start() override {
        NoAudioCodecSimplex::Start();  // loads the saved volume
        if (output_volume_ > kMaxOutputVolume) {
            ESP_LOGW(TAG, "Saved output volume %d capped to %d", output_volume_, kMaxOutputVolume);
            output_volume_ = kMaxOutputVolume;
        }
    }

    void SetOutputVolume(int volume) override {
        if (volume > kMaxOutputVolume) {
            ESP_LOGW(TAG, "Output volume %d capped to %d", volume, kMaxOutputVolume);
            volume = kMaxOutputVolume;
        }
        NoAudioCodecSimplex::SetOutputVolume(volume);
    }
};

class MotorDriver {
private:
    // Coast time (all inputs LOW) before energizing a new direction, so a running motor is
    // never reversed instantly: a hard reversal draws more than a start from rest and can
    // brown out the ESP32 on a weak supply.
    static constexpr int64_t kCoastUs = 100000;
    // A move starts only once the robot has been silent this long, so motor and speaker
    // current peaks do not add up; a move still waiting after kMaxWaitUs is cancelled.
    static constexpr int64_t kQuietUs = 800000;
    static constexpr int64_t kMaxWaitUs = 15000000;
    static constexpr uint32_t kSchedulerPollMs = 100;
    static constexpr uint32_t kSchedulerStackBytes = 3072;  // in PSRAM

    SemaphoreHandle_t mutex_ = nullptr;
    esp_timer_handle_t timer_ = nullptr;
    TaskHandle_t scheduler_ = nullptr;
    bool ready_ = false;
    bool moving_ = false;
    int64_t deadline_us_ = 0;
    bool active_levels_[4] = {};  // outputs while moving_
    int64_t stopped_us_ = 0;      // when a movement last ended (0 = never moved)
    uint32_t generation_ = 0;     // bumped by Stop(); cancels pending and coasting moves
    // The scheduled move, guarded by mutex_.
    bool pending_ = false;
    std::string pending_direction_;
    int pending_duration_ms_ = 0;
    int64_t pending_since_us_ = 0;

    static void AutoStop(void* arg) {
        static_cast<MotorDriver*>(arg)->StopFromTimer();
    }

    static void SchedulerEntry(void* arg) { static_cast<MotorDriver*>(arg)->SchedulerLoop(); }

    // Logical wheel motion per direction, with the mounting inversion applied.
    static bool LevelsFor(const std::string& direction, bool levels[4]) {
        if (direction == "forward") {
            levels[0] = true;  levels[1] = false; levels[2] = true;  levels[3] = false;
        } else if (direction == "backward") {
            levels[0] = false; levels[1] = true;  levels[2] = false; levels[3] = true;
        } else if (direction == "left") {
            levels[0] = false; levels[1] = true;  levels[2] = true;  levels[3] = false;
        } else if (direction == "right") {
            levels[0] = true;  levels[1] = false; levels[2] = false; levels[3] = true;
        } else {
            return false;
        }
        if (kInvertMotorA) {
            std::swap(levels[0], levels[1]);
        }
        if (kInvertMotorB) {
            std::swap(levels[2], levels[3]);
        }
        return true;
    }

    // Waits for each scheduled move until the robot is quiet, then starts it.
    void SchedulerLoop() {
        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            int64_t quiet_since_us = 0;
            while (true) {
                xSemaphoreTake(mutex_, portMAX_DELAY);
                const bool pending = pending_;
                const std::string direction = pending_direction_;
                const int duration_ms = pending_duration_ms_;
                const int64_t since_us = pending_since_us_;
                const uint32_t generation = generation_;
                xSemaphoreGive(mutex_);
                if (!pending) {
                    break;
                }

                const int64_t now_us = esp_timer_get_time();
                const bool speaking =
                    Application::GetInstance().GetDeviceState() == kDeviceStateSpeaking;
                if (speaking) {
                    quiet_since_us = 0;
                } else if (quiet_since_us == 0) {
                    quiet_since_us = now_us;
                }
                const bool timed_out = now_us - since_us > kMaxWaitUs;
                const bool quiet = quiet_since_us != 0 && now_us - quiet_since_us >= kQuietUs;
                const bool online = WifiManager::GetInstance().IsConnected();
                if (timed_out || quiet || !online) {
                    // Claim the move unless Stop() or a newer move replaced it meanwhile.
                    xSemaphoreTake(mutex_, portMAX_DELAY);
                    const bool claimed =
                        pending_ && generation_ == generation && pending_since_us_ == since_us;
                    if (claimed) {
                        pending_ = false;
                    }
                    xSemaphoreGive(mutex_);
                    if (claimed) {
                        if (!online) {
                            ESP_LOGW(TAG, "Scheduled move %s cancelled: Wi-Fi lost",
                                     direction.c_str());
                        } else if (!quiet) {
                            ESP_LOGW(TAG, "Scheduled move %s cancelled: robot kept speaking",
                                     direction.c_str());
                        } else {
                            const esp_err_t err = MoveNow(direction, duration_ms, generation);
                            ESP_LOGI(TAG, "Scheduled move %s %d ms after %lld ms wait: %s",
                                     direction.c_str(), duration_ms,
                                     static_cast<long long>((now_us - since_us) / 1000),
                                     esp_err_to_name(err));
                        }
                    }
                    continue;  // pick up a newer pending move, if any
                }
                vTaskDelay(pdMS_TO_TICKS(kSchedulerPollMs));
            }
        }
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
            stopped_us_ = esp_timer_get_time();
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
        if (xTaskCreatePinnedToCoreWithCaps(SchedulerEntry, "motor_sched", kSchedulerStackBytes,
                                            this, 2, &scheduler_, tskNO_AFFINITY,
                                            MALLOC_CAP_SPIRAM) != pdPASS) {
            scheduler_ = nullptr;
            ESP_LOGE(TAG, "Failed to create motor scheduler task; motors remain LOW");
            return;
        }
        ready_ = true;
        ESP_LOGI(TAG, "Motor outputs initialized LOW (GPIO11-14)");
    }

    // Validates and queues a move; it starts once the robot is quiet (see SchedulerLoop()).
    // A newer move replaces one still waiting; Stop() cancels it.
    esp_err_t Schedule(const std::string& direction, int duration_ms) {
        if (!ready_) {
            return ESP_ERR_INVALID_STATE;
        }
        bool levels[4];
        if (duration_ms < 500 || duration_ms > 5000 || !LevelsFor(direction, levels)) {
            return ESP_ERR_INVALID_ARG;
        }
        xSemaphoreTake(mutex_, portMAX_DELAY);
        pending_ = true;
        pending_direction_ = direction;
        pending_duration_ms_ = duration_ms;
        pending_since_us_ = esp_timer_get_time();
        xSemaphoreGive(mutex_);
        xTaskNotifyGive(scheduler_);
        return ESP_OK;
    }

private:
    // Starts a move now, unless Stop() ran since `generation` was read.
    esp_err_t MoveNow(const std::string& direction, int duration_ms, uint32_t generation) {
        bool levels[4];
        if (!LevelsFor(direction, levels)) {
            return ESP_ERR_INVALID_ARG;
        }

        if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        if (generation != generation_) {
            xSemaphoreGive(mutex_);
            return ESP_ERR_INVALID_STATE;  // stopped meanwhile
        }

        esp_timer_stop(timer_);
        const bool same_direction = moving_ && std::equal(levels, levels + 4, active_levels_);
        esp_err_t err = ESP_OK;
        if (!same_direction) {
            if (moving_) {
                SetAllLow();
                moving_ = false;
                deadline_us_ = 0;
                stopped_us_ = esp_timer_get_time();
            }
            // Coast with the mutex released, so Stop() and the auto-stop timer never wait on
            // it; a Stop() during the coast bumps generation_ and cancels this move.
            const int64_t wait_us = stopped_us_ + kCoastUs - esp_timer_get_time();
            if (stopped_us_ != 0 && wait_us > 0) {
                xSemaphoreGive(mutex_);
                vTaskDelay(pdMS_TO_TICKS((wait_us + 999) / 1000) + 1);
                if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
                    SetAllLow();
                    return ESP_ERR_TIMEOUT;
                }
                if (generation != generation_ || moving_) {
                    // Stopped, or another move started meanwhile: do not override it.
                    xSemaphoreGive(mutex_);
                    return ESP_ERR_INVALID_STATE;
                }
            }
            err = SetOutputs(levels[0], levels[1], levels[2], levels[3]);
        }
        if (err == ESP_OK) {
            deadline_us_ = esp_timer_get_time() + static_cast<int64_t>(duration_ms) * 1000;
            err = esp_timer_start_once(timer_, static_cast<uint64_t>(duration_ms) * 1000);
        }
        if (err == ESP_OK) {
            moving_ = true;
            std::copy(levels, levels + 4, active_levels_);
        } else {
            SetAllLow();
            if (moving_) {
                stopped_us_ = esp_timer_get_time();
            }
            moving_ = false;
            deadline_us_ = 0;
        }
        xSemaphoreGive(mutex_);
        return err;
    }

public:
    // Stops the motors and cancels a scheduled move.
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
        if (moving_) {
            stopped_us_ = esp_timer_get_time();
        }
        moving_ = false;
        deadline_us_ = 0;
        pending_ = false;
        ++generation_;
        xSemaphoreGive(mutex_);
        return ESP_OK;
    }
};

class HeartRateSensor {
private:
    static constexpr uint8_t kRegFifoWritePtr = 0x04;
    static constexpr uint8_t kRegFifoReadPtr = 0x06;
    static constexpr uint8_t kRegFifoData = 0x07;
    static constexpr size_t kFifoDepth = 32;
    // 0x0A: 100 samples/s; 0x08: 4-sample average -> one FIFO entry every 40 ms.
    static constexpr uint64_t kSamplePeriodUs = 40000;
    // A steady reading stays reportable this long after the last steady 4-interval window,
    // while the finger stays on the sensor (a single missed or extra beat breaks a window).
    static constexpr uint64_t kHeldReadingUs = 10000000;
    static constexpr uint64_t kWarnIntervalUs = 10000000;

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t device_ = nullptr;
    TaskHandle_t task_ = nullptr;
    SemaphoreHandle_t mutex_ = nullptr;
    std::string init_error_ = "MAX30102 is not initialized";  // guarded by init_mutex_
    SemaphoreHandle_t init_mutex_ = nullptr;  // serializes Init() between boot and tool retries
    std::atomic<bool> ready_{false};
    int64_t last_init_attempt_us_ = 0;
    char scan_line_[112 * 5 + 8] = {};  // I2C scan result after a failed init

    // Beat detector state (sensor task, under mutex_). See UpdateBeat().
    int32_t ir_history_[3] = {};  // last three IR samples, oldest first
    size_t ir_history_count_ = 0;
    bool trend_ready_ = false;
    float slope_trend_ = 0;  // slow average of the 80 ms slope (baseline drift)
    int32_t drop_peak_ = 0;  // strongest recent systolic drop, decays ~2 s
    int32_t previous_slope_ = 0;
    uint32_t beat_intervals_ms_[4] = {};
    size_t interval_count_ = 0;
    size_t interval_next_ = 0;
    uint64_t last_beat_us_ = 0;
    int latest_bpm_ = 0;  // 0 unless the last 4 intervals are steady
    int held_bpm_ = 0;    // last steady reading, cleared when the finger is removed
    uint64_t held_us_ = 0;
    bool finger_ = false;
    uint32_t last_ir_ = 0;        // reported by the tool
    uint32_t total_samples_ = 0;  // samples read since the task started

    // Sensor task only, no lock: rate-limited warnings.
    uint32_t fifo_full_count_ = 0;
    uint32_t i2c_error_count_ = 0;
    uint64_t last_warn_us_ = 0;
    uint64_t last_sample_us_ = 0;  // keeps batch timestamps strictly increasing

    bool ReadRegister(uint8_t reg, uint8_t* data, size_t length) {
        return i2c_master_transmit_receive(device_, &reg, 1, data, length, 100) == ESP_OK;
    }

    bool WriteRegister(uint8_t reg, uint8_t value) {
        uint8_t buffer[] = {reg, value};
        return i2c_master_transmit(device_, buffer, sizeof(buffer), 100) == ESP_OK;
    }

    static void SensorTask(void* arg) { static_cast<HeartRateSensor*>(arg)->Run(); }

    // This chip sets bits 23:18 on real samples (e.g. IR 3B FF FF), so keep only bits 17:0.
    static uint32_t Field18(const uint8_t* bytes) {
        return ((static_cast<uint32_t>(bytes[0]) << 16) | (static_cast<uint32_t>(bytes[1]) << 8) |
                bytes[2]) &
               0x3FFFF;
    }

    // Beat detection on the raw IR signal. A resting finger reads ~62000 IR; each heartbeat is
    // a sharp drop (~100-200 counts within 80 ms, the systolic upstroke absorbing more IR)
    // followed by a slow rise, so the pulse is ~0.3% of DC. A beat is counted when the
    // detrended 80 ms slope falls below -threshold, where threshold is half the strongest
    // recent drop (decaying over ~2 s), at least DC/2000 and 20 counts. A BPM is reported only
    // when the last 4 intervals agree within 25%; motion and drift give irregular intervals.
    void UpdateBeat(uint32_t ir, uint64_t now_us) {
        const bool finger = ir > kFingerThreshold;
        xSemaphoreTake(mutex_, portMAX_DELAY);
        const bool finger_changed = finger != finger_;
        finger_ = finger;

        if (!finger) {
            ir_history_count_ = 0;
            trend_ready_ = false;
            slope_trend_ = 0;
            drop_peak_ = 0;
            previous_slope_ = 0;
            interval_count_ = 0;
            interval_next_ = 0;
            last_beat_us_ = 0;
            latest_bpm_ = 0;
            held_bpm_ = 0;
            held_us_ = 0;
        } else {
            ir_history_[0] = ir_history_[1];
            ir_history_[1] = ir_history_[2];
            ir_history_[2] = static_cast<int32_t>(ir);
            if (ir_history_count_ < 3) {
                ++ir_history_count_;
            }
        }

        if (finger && ir_history_count_ == 3) {
            const int32_t raw_slope = ir_history_[2] - ir_history_[0];
            if (!trend_ready_) {
                slope_trend_ = static_cast<float>(raw_slope);  // first slope after placement
                trend_ready_ = true;
            }
            slope_trend_ += (static_cast<float>(raw_slope) - slope_trend_) / 32.0f;
            const int32_t slope = raw_slope - static_cast<int32_t>(slope_trend_);
            drop_peak_ -= drop_peak_ / 50;
            if (-slope > drop_peak_) {
                drop_peak_ = -slope;
            }
            const int32_t floor = std::max<int32_t>(static_cast<int32_t>(ir / 2000), 20);
            const int32_t drop_threshold = std::max(floor, drop_peak_ / 2);

            if (previous_slope_ > -drop_threshold && slope <= -drop_threshold &&
                (last_beat_us_ == 0 || now_us - last_beat_us_ >= 300000)) {
                if (last_beat_us_ != 0) {
                    uint64_t interval_us = now_us - last_beat_us_;
                    if (interval_us >= 300000 && interval_us <= 2000000) {
                        beat_intervals_ms_[interval_next_] =
                            static_cast<uint32_t>(interval_us / 1000);
                        interval_next_ = (interval_next_ + 1) % 4;
                        if (interval_count_ < 4) {
                            ++interval_count_;
                        }
                        if (interval_count_ == 4) {
                            uint64_t total_ms = 0;
                            uint32_t min_ms = beat_intervals_ms_[0];
                            uint32_t max_ms = beat_intervals_ms_[0];
                            for (uint32_t interval : beat_intervals_ms_) {
                                total_ms += interval;
                                min_ms = std::min(min_ms, interval);
                                max_ms = std::max(max_ms, interval);
                            }
                            // Steady rhythm only: the longest interval at most 25% above the
                            // shortest. Otherwise report nothing rather than a wrong BPM.
                            latest_bpm_ =
                                max_ms * 4 <= min_ms * 5
                                    ? static_cast<int>((4ULL * 60000 + total_ms / 2) / total_ms)
                                    : 0;
                            if (latest_bpm_ > 0) {
                                held_bpm_ = latest_bpm_;
                                held_us_ = now_us;
                            }
                        }
                    } else {
                        interval_count_ = 0;
                        interval_next_ = 0;
                    }
                }
                last_beat_us_ = now_us;
            }
            previous_slope_ = slope;

            if (last_beat_us_ != 0 && now_us - last_beat_us_ > kHeartRateStaleUs) {
                interval_count_ = 0;
                interval_next_ = 0;
                last_beat_us_ = 0;
                latest_bpm_ = 0;
            }
        }

        last_ir_ = ir;
        ++total_samples_;
        xSemaphoreGive(mutex_);
        if (finger_changed) {
            ESP_LOGI(TAG, "Heart rate: finger %s (IR %lu)", finger ? "detected" : "removed",
                     static_cast<unsigned long>(ir));
        }
    }

    void WarnIfNeeded(uint64_t now_us) {
        if ((fifo_full_count_ == 0 && i2c_error_count_ == 0) ||
            (last_warn_us_ != 0 && now_us - last_warn_us_ < kWarnIntervalUs)) {
            return;
        }
        ESP_LOGW(TAG, "MAX30102: %lu full-FIFO drains, %lu I2C errors since the last warning",
                 static_cast<unsigned long>(fifo_full_count_),
                 static_cast<unsigned long>(i2c_error_count_));
        fifo_full_count_ = 0;
        i2c_error_count_ = 0;
        last_warn_us_ = now_us;
    }

    // On this sensor, rewriting the FIFO pointer registers does not empty a full FIFO: it stays
    // full (FIFO_WR_PTR == FIFO_RD_PTR, OVF_COUNTER climbing) and drops every new sample until
    // entries are read from FIFO_DATA. So the FIFO is never cleared by register writes; a full
    // FIFO (expected once at boot) is drained by reading all 32 entries.
    void Run() {
        while (true) {
            PollFifo();
            WarnIfNeeded(esp_timer_get_time());
            // The 32-entry FIFO holds 1.28 s at one entry per 40 ms, so a 100 ms poll never
            // loses samples; PollFifo() timestamps each entry of a batch.
            vTaskDelay(pdMS_TO_TICKS(kPollMs));
        }
    }

    // Reads every available sample. A non-zero OVF_COUNTER means a full FIFO (FIFO_WR_PTR ==
    // FIFO_RD_PTR): its 32 stale entries are read and discarded.
    void PollFifo() {
        uint8_t regs[3] = {};  // 0x04 FIFO_WR_PTR, 0x05 OVF_COUNTER, 0x06 FIFO_RD_PTR
        if (!ReadRegister(kRegFifoWritePtr, regs, sizeof(regs))) {
            ++i2c_error_count_;
            return;
        }
        const uint8_t write_ptr = regs[0] & 0x1F;
        const bool full = (regs[1] & 0x1F) != 0;
        const uint8_t read_ptr = regs[2] & 0x1F;
        const size_t available = full ? kFifoDepth : ((write_ptr - read_ptr) & 0x1F);
        if (available == 0) {
            return;
        }

        const uint64_t now_us = esp_timer_get_time();
        for (size_t i = 0; i < available; ++i) {
            uint8_t sample[6];  // RED[0..2], IR[3..5]
            if (!ReadRegister(kRegFifoData, sample, sizeof(sample))) {
                ++i2c_error_count_;
                break;
            }
            if (full) {
                continue;  // stale entries from before an unknown gap: drained, not analysed
            }
            // Entries are 40 ms apart and the newest was just written; stamp each one so a
            // batch read does not collapse several samples onto one timestamp.
            uint64_t sample_us = now_us - (available - 1 - i) * kSamplePeriodUs;
            if (sample_us <= last_sample_us_) {
                sample_us = last_sample_us_ + 1;
            }
            last_sample_us_ = sample_us;
            UpdateBeat(Field18(sample + 3), sample_us);
        }
        if (full) {
            ++fifo_full_count_;
        }
    }

    static constexpr gpio_num_t kSdaPin = GPIO_NUM_8;
    static constexpr gpio_num_t kSclPin = GPIO_NUM_9;
    static constexpr uint32_t kI2cSpeedHz = 100000;
    static constexpr int kI2cTimeoutMs = 20;
    static constexpr int kBootInitAttempts = 3;
    static constexpr uint64_t kInitRetryUs = 2000000;
    static constexpr uint32_t kTaskStackBytes = 5120;  // in PSRAM, saves internal SRAM
    static constexpr uint32_t kPollMs = 100;

    void ReleaseI2c() {
        if (device_ != nullptr) {
            i2c_master_bus_rm_device(device_);
            device_ = nullptr;
        }
        if (bus_ != nullptr) {
            i2c_del_master_bus(bus_);
            bus_ = nullptr;
        }
    }

    // After a failed init attempt: line levels and an address scan, to tell wiring faults apart.
    void LogBusDiagnostics() {
        int sda = 0;
        int scl = 0;
        if (bus_ != nullptr) {
            // The I2C driver leaves both pins open-drain, input-enabled, with internal pull-ups.
            sda = gpio_get_level(kSdaPin);
            scl = gpio_get_level(kSclPin);
        } else {
            gpio_config_t config = {};
            config.pin_bit_mask = (1ULL << kSdaPin) | (1ULL << kSclPin);
            config.mode = GPIO_MODE_INPUT;
            config.pull_up_en = GPIO_PULLUP_ENABLE;
            config.pull_down_en = GPIO_PULLDOWN_DISABLE;
            config.intr_type = GPIO_INTR_DISABLE;
            gpio_config(&config);
            esp_rom_delay_us(50);
            sda = gpio_get_level(kSdaPin);
            scl = gpio_get_level(kSclPin);
            gpio_reset_pin(kSdaPin);
            gpio_reset_pin(kSclPin);
        }
        ESP_LOGW(TAG, "MAX30102 diag: SDA(GPIO8)=%d SCL(GPIO9)=%d with internal pull-ups", sda,
                 scl);
        if (bus_ == nullptr) {
            ESP_LOGW(TAG, "MAX30102 diag: I2C scan skipped, bus was not created");
            return;
        }
        if (sda == 0 || scl == 0) {
            ESP_LOGW(TAG, "MAX30102 diag: I2C scan skipped, a line is held LOW");
            return;
        }
        int len = 0;
        scan_line_[0] = '\0';
        for (uint16_t address = 0x08; address <= 0x77; ++address) {
            if (i2c_master_probe(bus_, address, kI2cTimeoutMs) == ESP_OK &&
                len < static_cast<int>(sizeof(scan_line_))) {
                len += snprintf(scan_line_ + len, sizeof(scan_line_) - len, " 0x%02X", address);
            }
        }
        ESP_LOGW(TAG, "MAX30102 diag: I2C scan 0x08-0x77 ACK:%s", len > 0 ? scan_line_ : " none");
    }

    // One probe/configure attempt. On failure, sets init_error_ and leaves bus_/device_ for the
    // caller to diagnose and release.
    bool TryInitOnce() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_1,
            .sda_io_num = kSdaPin,
            .scl_io_num = kSclPin,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = 1},
        };
        esp_err_t err = i2c_new_master_bus(&bus_config, &bus_);
        if (err != ESP_OK) {
            bus_ = nullptr;
            init_error_ =
                "Failed to initialize MAX30102 I2C bus 2: " + std::string(esp_err_to_name(err));
            return false;
        }

        err = i2c_master_probe(bus_, kMax30102Address, kI2cTimeoutMs);
        if (err != ESP_OK) {
            init_error_ = "MAX30102 not found at I2C address 0x57";
            return false;
        }

        i2c_device_config_t device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = kMax30102Address,
            .scl_speed_hz = kI2cSpeedHz,
            .scl_wait_us = 0,
            .flags = {.disable_ack_check = 0},
        };
        err = i2c_master_bus_add_device(bus_, &device_config, &device_);
        if (err != ESP_OK) {
            device_ = nullptr;
            init_error_ = "Failed to add MAX30102 I2C device: " + std::string(esp_err_to_name(err));
            return false;
        }
        uint8_t part_id = 0;
        if (!ReadRegister(0xFF, &part_id, 1)) {
            init_error_ = "MAX30102 at 0x57 did not answer the part-ID read";
            return false;
        }
        if (part_id != kMax30102PartId) {
            // Something ACKs at 0x57 but is not a MAX30102.
            ESP_LOGW(TAG, "MAX30102 diag: device at 0x57 has part ID 0x%02X, expected 0x%02X",
                     part_id, kMax30102PartId);
            char message[80];
            snprintf(message, sizeof(message),
                     "Device at I2C address 0x57 has part ID 0x%02X, expected 0x15", part_id);
            init_error_ = message;
            return false;
        }

        if (!WriteRegister(0x09, 0x40)) {
            init_error_ = "Failed to reset MAX30102";
            return false;
        }
        // Wait for the RESET bit (0x09 bit 6) to self-clear, up to 100 ms.
        bool reset_done = false;
        for (int i = 0; i < 10 && !reset_done; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
            uint8_t mode = 0;
            reset_done = ReadRegister(0x09, &mode, 1) && (mode & 0x40) == 0;
        }
        if (!reset_done) {
            init_error_ = "MAX30102 reset bit did not clear within 100 ms";
            return false;
        }
        // Reading INT_STATUS_1 and INT_STATUS_2 clears them (including PWR_RDY).
        uint8_t status = 0;
        if (!ReadRegister(0x00, &status, 1) || !ReadRegister(0x01, &status, 1)) {
            init_error_ = "Failed to read MAX30102 interrupt status";
            return false;
        }
        if (!WriteRegister(0x08, 0x4F) || !WriteRegister(0x09, 0x03) ||
            !WriteRegister(0x0A, kSpo2Config) || !WriteRegister(0x0C, kLedCurrent) ||
            !WriteRegister(0x0D, kLedCurrent) || !WriteRegister(kRegFifoWritePtr, 0) ||
            !WriteRegister(0x05, 0) || !WriteRegister(kRegFifoReadPtr, 0)) {
            init_error_ = "Failed to configure MAX30102";
            return false;
        }
        return true;
    }

    // One full attempt; the caller holds init_mutex_. A failed attempt logs bus diagnostics and
    // releases the I2C bus and device handles so the next attempt starts clean.
    bool Init() {
        last_init_attempt_us_ = esp_timer_get_time();
        if (!TryInitOnce()) {
            LogBusDiagnostics();
            ReleaseI2c();
            return false;
        }
        if (task_ == nullptr &&
            xTaskCreatePinnedToCoreWithCaps(SensorTask, "heart_rate", kTaskStackBytes, this, 1,
                                            &task_, tskNO_AFFINITY,
                                            MALLOC_CAP_SPIRAM) != pdPASS) {
            task_ = nullptr;
            init_error_ = "Failed to create MAX30102 sensor task";
            ReleaseI2c();
            return false;
        }
        init_error_.clear();
        ready_ = true;
        ESP_LOGI(
            TAG, "MAX30102 found at 0x57 (I2C %lu Hz); sensor task stack=%lu bytes (PSRAM), priority=1",
            static_cast<unsigned long>(kI2cSpeedHz), static_cast<unsigned long>(kTaskStackBytes));
        return true;
    }

public:
    HeartRateSensor() {
        mutex_ = xSemaphoreCreateMutex();
        init_mutex_ = xSemaphoreCreateMutex();
        if (mutex_ == nullptr || init_mutex_ == nullptr) {
            init_error_ = "Failed to create MAX30102 mutexes";
            ESP_LOGE(TAG, "%s", init_error_.c_str());
            return;
        }
        xSemaphoreTake(init_mutex_, portMAX_DELAY);
        const int64_t start_us = esp_timer_get_time();
        for (int attempt = 1; attempt <= kBootInitAttempts; ++attempt) {
            if (Init()) {
                break;
            }
            ESP_LOGW(TAG, "MAX30102 init attempt %d/%d failed: %s", attempt, kBootInitAttempts,
                     init_error_.c_str());
            if (attempt < kBootInitAttempts) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        if (!ready_) {
            ESP_LOGE(TAG,
                     "MAX30102 unavailable after %d attempts (%lld ms); the tool retries later",
                     kBootInitAttempts, (esp_timer_get_time() - start_us) / 1000);
        }
        xSemaphoreGive(init_mutex_);
    }

    // Non-blocking: runs on the main event loop, so it only reads the sampler task's state.
    // If the sensor is not ready, it makes at most one init attempt per 2 s.
    ToolResult GetHeartRate() {
        if (!ready_) {
            if (mutex_ == nullptr || init_mutex_ == nullptr) {
                return std::unexpected(init_error_);
            }
            if (xSemaphoreTake(init_mutex_, 0) != pdTRUE) {
                return std::unexpected(std::string(
                    "Heart-rate sensor initialization is in progress; ask again shortly."));
            }
            if (!ready_ && esp_timer_get_time() - last_init_attempt_us_ >= kInitRetryUs) {
                if (!Init()) {
                    ESP_LOGW(TAG, "MAX30102 init retry failed: %s", init_error_.c_str());
                }
            }
            const bool ready = ready_;
            const std::string error = init_error_;
            xSemaphoreGive(init_mutex_);
            if (!ready) {
                return std::unexpected("Heart-rate sensor unavailable (last init error: " + error +
                                       "). Check the MAX30102 wiring; the robot retries at most "
                                       "every 2 seconds.");
            }
        }

        const uint64_t now_us = esp_timer_get_time();
        xSemaphoreTake(mutex_, portMAX_DELAY);
        const bool fresh =
            latest_bpm_ > 0 && last_beat_us_ != 0 && now_us - last_beat_us_ <= kHeartRateStaleUs;
        const int bpm = latest_bpm_;
        // A missed or extra beat breaks the 4-interval window for a few seconds; while the
        // finger stays on, the last steady reading is still a valid answer for 10 s.
        const bool held = !fresh && finger_ && held_bpm_ > 0 && now_us - held_us_ <= kHeldReadingUs;
        const int held_bpm = held_bpm_;
        const size_t beats = interval_count_;
        const uint32_t last_ir = last_ir_;
        const uint32_t samples = total_samples_;
        xSemaphoreGive(mutex_);

        if (fresh || held) {
            return "BPM: " + std::to_string(fresh ? bpm : held_bpm) +
                   ". This is a reference value, not medical.";
        }
        const char* finger_state = samples == 0                 ? "no sensor data yet"
                                   : last_ir > kFingerThreshold ? "finger detected"
                                                                : "no finger detected";
        return "No fresh reading yet (" + std::string(finger_state) + "; last IR " +
               std::to_string(last_ir) + ", finger needs above " +
               std::to_string(kFingerThreshold) + "; samples read " + std::to_string(samples) +
               "). Ask the user to rest a finger on the sensor, keep still, "
               "and ask again in about 5 seconds. Beats detected: " +
               std::to_string(beats) + "/4. This is a reference value, not medical.";
    }
};

// Weather for one fixed place (kWeatherPlace). A low-priority task with its stack in PSRAM
// fetches Open-Meteo over HTTPS every 15 min (1 min after a failure) into a cache; GetText()
// only reads the cache, so the MCP tool never waits on the network.
class WeatherService {
private:
    static constexpr uint32_t kStackBytes = 8192;
    static constexpr uint32_t kRefreshMs = 15 * 60 * 1000;
    static constexpr uint32_t kRetryMs = 60 * 1000;
    static constexpr uint32_t kWifiPollMs = 5000;
    static constexpr int kHttpTimeoutMs = 10000;
    static constexpr size_t kMaxBodyBytes = 4096;

    SemaphoreHandle_t mutex_ = nullptr;
    TaskHandle_t task_ = nullptr;
    // Guarded by mutex_.
    bool has_report_ = false;
    robot_emo::WeatherReport report_;
    int64_t fetched_us_ = 0;
    std::string last_error_;
    bool stack_logged_ = false;  // weather task only

    static void TaskEntry(void* arg) { static_cast<WeatherService*>(arg)->Run(); }

    std::expected<std::string, std::string> Download() {
        auto http = Board::GetInstance().GetNetwork()->CreateHttp(3);
        if (http == nullptr) {
            return std::unexpected(std::string("no HTTP client"));
        }
        http->SetTimeout(kHttpTimeoutMs);
        if (auto opened = http->Open("GET", kWeatherUrl); !opened) {
            return std::unexpected("connect failed: " + opened.error().ToString());
        }
        auto status = http->GetStatusCode();
        if (!status) {
            http->Close();
            return std::unexpected("no response: " + status.error().ToString());
        }
        if (*status != 200) {
            http->Close();
            return std::unexpected("HTTP status " + std::to_string(*status));
        }
        std::string body;
        char buffer[512];
        while (true) {
            auto read = http->Read(buffer, sizeof(buffer));
            if (!read) {
                http->Close();
                return std::unexpected("read failed: " + read.error().ToString());
            }
            if (*read <= 0) {
                break;
            }
            body.append(buffer, static_cast<size_t>(*read));
            if (body.size() > kMaxBodyBytes) {
                http->Close();
                return std::unexpected(std::string("response larger than 4 KB"));
            }
        }
        http->Close();
        return body;
    }

    // One fetch; returns the delay before the next one.
    uint32_t FetchOnce() {
        const int64_t start_us = esp_timer_get_time();
        auto body = Download();
        std::expected<robot_emo::WeatherReport, std::string> report =
            body ? robot_emo::ParseOpenMeteo(*body)
                 : std::expected<robot_emo::WeatherReport, std::string>(
                       std::unexpected(body.error()));
        xSemaphoreTake(mutex_, portMAX_DELAY);
        if (report) {
            report_ = *report;
            has_report_ = true;
            fetched_us_ = esp_timer_get_time();
            last_error_.clear();
        } else {
            last_error_ = report.error();  // keep the previous report, if any
        }
        xSemaphoreGive(mutex_);

        if (!report) {
            ESP_LOGW(TAG, "Weather fetch failed: %s; retrying in 1 min", report.error().c_str());
            return kRetryMs;
        }
        const char* condition = robot_emo::WmoDescription(report->weather_code);
        ESP_LOGI(TAG, "Weather: %.1f C, %s (fetched in %lld ms)", report->temperature_c,
                 condition != nullptr ? condition : "unknown",
                 static_cast<long long>((esp_timer_get_time() - start_us) / 1000));
        if (!stack_logged_) {
            stack_logged_ = true;
            ESP_LOGI(TAG, "Weather task stack: %u of %lu bytes never used",
                     static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
                     static_cast<unsigned long>(kStackBytes));
        }
        return kRefreshMs;
    }

    void Run() {
        while (true) {
            while (!WifiManager::GetInstance().IsConnected()) {
                vTaskDelay(pdMS_TO_TICKS(kWifiPollMs));
            }
            vTaskDelay(pdMS_TO_TICKS(FetchOnce()));
        }
    }

public:
    WeatherService() {
        mutex_ = xSemaphoreCreateMutex();
        if (mutex_ == nullptr) {
            ESP_LOGE(TAG, "Weather: failed to create mutex; weather disabled");
        }
    }

    // Starts the fetch task once. On failure the robot runs without weather.
    void Start() {
        if (mutex_ == nullptr || task_ != nullptr) {
            return;
        }
        if (xTaskCreatePinnedToCoreWithCaps(TaskEntry, "weather", kStackBytes, this, 1, &task_,
                                            tskNO_AFFINITY, MALLOC_CAP_SPIRAM) != pdPASS) {
            task_ = nullptr;
            xSemaphoreTake(mutex_, portMAX_DELAY);
            last_error_ = "weather task could not be created";
            xSemaphoreGive(mutex_);
            ESP_LOGE(TAG, "Weather: failed to create task; weather disabled");
        }
    }

    // Non-blocking: reads the cache only.
    ToolResult GetText() {
        if (mutex_ == nullptr) {
            return std::unexpected(std::string("Weather is unavailable on this robot"));
        }
        xSemaphoreTake(mutex_, portMAX_DELAY);
        const bool has_report = has_report_;
        const robot_emo::WeatherReport report = report_;
        const int64_t fetched_us = fetched_us_;
        const std::string error = last_error_;
        xSemaphoreGive(mutex_);

        if (!has_report) {
            // No error yet: either Wi-Fi is down, or it is up and the first fetch is running.
            const std::string reason = !error.empty() ? ": " + error
                                       : WifiManager::GetInstance().IsConnected()
                                           ? std::string(", the first fetch is in progress")
                                           : std::string(", waiting for Wi-Fi");
            return std::unexpected(
                "Weather not fetched yet" + reason +
                ". Tell the user the weather is not available yet and to ask again in a minute.");
        }
        const int64_t age_s = (esp_timer_get_time() - fetched_us) / 1000000;
        return robot_emo::FormatWeather(report, kWeatherPlace, age_s);
    }
};

}  // namespace

class RobotEmoBoard : public WifiBoard {
private:
    MotorDriver motor_driver_;
    HeartRateSensor heart_rate_sensor_;
    WeatherService weather_;
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
            "The movement starts once the robot has finished speaking (to protect its power supply) "
            "and automatically stops after duration_ms (500-5000; default 3000).",
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

                esp_err_t err = motor_driver_.Schedule(direction, duration_ms);
                if (err != ESP_OK) {
                    const std::string result = std::string("error: ") + esp_err_to_name(err);
                    ESP_LOGI(TAG,
                             "MCP self.robot.move args={direction:%s,duration_ms:%d} result=%s",
                             direction.c_str(), duration_ms, result.c_str());
                    return std::unexpected(std::string("Motor move failed: ") + esp_err_to_name(err));
                }
                const std::string result = "will move " + direction + " for " +
                                           std::to_string(duration_ms) +
                                           " ms as soon as the robot stops speaking";
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

        mcp_server.AddTool(
            "self.weather.get",
            "Current weather plus today's and tomorrow's forecast for where the robot is (Thu Duc "
            "City, Ho Chi Minh City). Use for any question about the weather, temperature, rain "
            "or tomorrow's weather here. Returns immediately from a cache refreshed every 15 "
            "minutes; mention it if the data is marked STALE.",
            PropertyList(), [this](const PropertyList&) -> ToolResult {
                ToolResult result = weather_.GetText();
                if (!result) {
                    ESP_LOGI(TAG, "MCP self.weather.get args={} result=error: %s",
                             result.error().c_str());
                    return result;
                }
                const std::string& text = std::get<std::string>(*result);
                ESP_LOGI(TAG, "MCP self.weather.get args={} result=%s", text.c_str());
                return result;
            });
    }

public:
    RobotEmoBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        RegisterMcpTools();
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
        weather_.Start();  // waits for Wi-Fi in its own task
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
        static RobotEmoAudioCodec audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(RobotEmoBoard);

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
#include <esp_rom_sys.h>
#include <esp_heap_caps.h>  // DIAGNOSTIC, REMOVE LATER (SRAM log)
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
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
constexpr uint32_t kFingerThreshold = 50000;
constexpr uint64_t kHeartRateStaleUs = 2500000;
// MAX30102 LED1 (red) and LED2 (IR) pulse amplitude, registers 0x0C/0x0D: 0.2 mA/LSB, 0x3C = 12 mA.
constexpr uint8_t kLedCurrent = 0x3C;
// DIAGNOSTIC, REMOVE LATER: log raw IR samples ("HRRAW,...") for up to 20 s per finger placement.
constexpr bool kHeartRateRawCapture = true;
// DIAGNOSTIC, REMOVE LATER: observation build for the MAX30102 FIFO. true: no overflow recovery;
// log HROBS for 40 polls, then validated one-sample-per-poll reads (HRSAMPLE) and keep that path.
// false: the batch read with full-FIFO recovery.
constexpr bool kHrObserve = true;

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
        // The table above is logical wheel motion; apply mounting inversion here only.
        if (kInvertMotorA) {
            std::swap(levels[0], levels[1]);
        }
        if (kInvertMotorB) {
            std::swap(levels[2], levels[3]);
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
    static constexpr uint8_t kRegOvfCounter = 0x05;
    static constexpr uint8_t kRegFifoReadPtr = 0x06;
    static constexpr size_t kFifoDepth = 32;
    // 0x0A: 100 samples/s; 0x08: 4-sample average -> one FIFO entry every 40 ms.
    static constexpr uint64_t kSamplePeriodUs = 40000;
    static constexpr uint8_t kRegFifoData = 0x07;

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t device_ = nullptr;
    TaskHandle_t task_ = nullptr;
    SemaphoreHandle_t mutex_ = nullptr;
    std::string init_error_ = "MAX30102 is not initialized";  // guarded by init_mutex_
    SemaphoreHandle_t init_mutex_ = nullptr;  // serializes Init() between boot and tool retries
    std::atomic<bool> ready_{false};
    int64_t last_init_attempt_us_ = 0;
    char scan_line_[112 * 5 + 8] = {};  // DIAGNOSTIC, REMOVE LATER: I2C scan result

    int32_t ir_baseline_ = 0;
    int32_t previous_ac_ = 0;
    uint32_t beat_intervals_ms_[4] = {};
    size_t interval_count_ = 0;
    size_t interval_next_ = 0;
    uint64_t last_beat_us_ = 0;
    int latest_bpm_ = 0;
    uint32_t last_ir_ = 0;         // guarded by mutex_; reported by the tool
    uint32_t total_samples_ = 0;   // guarded by mutex_; samples read since the task started

    // DIAGNOSTIC, REMOVE LATER: HRALIVE/HROBS/HRSAMPLE state. Sensor task only, no lock.
    static constexpr uint64_t kAliveIntervalUs = 2000000;
    static constexpr int kObservePolls = 40;
    static constexpr uint32_t kObservePollMs = 100;
    static constexpr int64_t kSampleCheckUs = 5000000;
    static constexpr int kSampleLogCount = 12;
    uint64_t alive_start_us_ = 0;
    uint32_t alive_samples_ = 0;
    uint32_t alive_i2c_errors_ = 0;
    uint32_t alive_polls_ = 0;
    uint32_t alive_data_polls_ = 0;
    uint32_t alive_invalid_ = 0;     // samples whose RED or IR top byte is above 0x03
    uint32_t alive_ovf_polls_ = 0;   // polls that read OVF_COUNTER != 0
    uint8_t alive_ovf_last_ = 0;
    uint32_t alive_recoveries_ = 0;  // recovery path only (kHrObserve == false)
    uint32_t recoveries_total_ = 0;
    uint32_t samples_total_ = 0;
    uint32_t invalid_total_ = 0;
    uint8_t alive_raw_[6] = {};  // last FIFO sample as read: RED[0..2], IR[3..5]
    uint8_t alive_write_ptr_ = 0;
    uint8_t alive_read_ptr_ = 0;
    int samples_logged_ = 0;  // HRSAMPLE: first 12 raw samples
    char alive_line_[384] = {};
    int64_t init_done_us_ = 0;  // when Init() last configured the sensor and cleared the FIFO
    uint64_t last_sample_us_ = 0;  // recovery path: keeps batch timestamps strictly increasing

    // DIAGNOSTIC, REMOVE LATER: HRDIAG/HRRAW signal logging. Touched only by the sensor task,
    // so it needs no lock.
    static constexpr uint32_t kDiagMinIr = 10000;
    static constexpr uint64_t kRawCaptureUs = 20000000;
    static constexpr uint64_t kRawRearmUs = 3000000;
    static constexpr size_t kRawMaxSamples = 40;  // per 1 s line; the FIFO delivers ~25/s
    uint64_t diag_window_start_us_ = 0;  // 0: no 1 s window open
    uint32_t diag_ir_min_ = 0;
    uint32_t diag_ir_max_ = 0;
    int32_t diag_ac_min_ = 0;
    int32_t diag_ac_max_ = 0;
    bool diag_has_ac_ = false;
    uint32_t diag_samples_ = 0;
    bool raw_armed_ = true;
    uint64_t raw_start_us_ = 0;
    uint64_t below_since_us_ = 0;
    uint64_t raw_line_start_us_ = 0;
    uint32_t raw_samples_[kRawMaxSamples] = {};
    size_t raw_count_ = 0;
    char raw_line_[kRawMaxSamples * 8 + 32] = {};

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
        const bool finger = ir > kFingerThreshold;
        int32_t ac = 0;
        int32_t peak_threshold = 0;
        xSemaphoreTake(mutex_, portMAX_DELAY);

        if (!finger) {
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
            ac = static_cast<int32_t>(ir) - ir_baseline_;
            const int32_t threshold = ir_baseline_ / 200;
            peak_threshold = threshold > 200 ? threshold : 200;

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

        last_ir_ = ir;
        ++total_samples_;
        const int32_t baseline = ir_baseline_;
        const size_t beats = interval_count_;
        xSemaphoreGive(mutex_);
        UpdateDiagnostics(ir, now_us, finger, baseline, ac, peak_threshold, beats);
    }

    // DIAGNOSTIC, REMOVE LATER: while IR > 10000, logs one HRDIAG summary line per second and,
    // if kHeartRateRawCapture, one HRRAW line per second with every IR sample of that second.
    // HRRAW covers at most 20 s after the finger is first seen; it re-arms after IR stays at
    // or below 10000 for 3 s. HRRAW's <ms> is the line's first sample, in ms since capture start.
    void UpdateDiagnostics(uint32_t ir, uint64_t now_us, bool finger, int32_t baseline,
                           int32_t ac, int32_t peak_threshold, size_t beats) {
        if (ir <= kDiagMinIr) {
            diag_window_start_us_ = 0;
            raw_count_ = 0;
            if (below_since_us_ == 0) {
                below_since_us_ = now_us;
            }
            if (!raw_armed_ && now_us - below_since_us_ >= kRawRearmUs) {
                raw_armed_ = true;
            }
            return;
        }
        below_since_us_ = 0;

        if (diag_window_start_us_ == 0) {
            diag_window_start_us_ = now_us;
            diag_ir_min_ = ir;
            diag_ir_max_ = ir;
            diag_has_ac_ = false;
            diag_samples_ = 0;
        }
        diag_ir_min_ = ir < diag_ir_min_ ? ir : diag_ir_min_;
        diag_ir_max_ = ir > diag_ir_max_ ? ir : diag_ir_max_;
        if (finger) {
            if (!diag_has_ac_) {
                diag_ac_min_ = ac;
                diag_ac_max_ = ac;
                diag_has_ac_ = true;
            }
            diag_ac_min_ = ac < diag_ac_min_ ? ac : diag_ac_min_;
            diag_ac_max_ = ac > diag_ac_max_ ? ac : diag_ac_max_;
        }
        ++diag_samples_;

        if (kHeartRateRawCapture) {
            if (raw_armed_) {
                raw_armed_ = false;
                raw_start_us_ = now_us;
                raw_count_ = 0;
            }
            if (now_us - raw_start_us_ <= kRawCaptureUs && raw_count_ < kRawMaxSamples) {
                if (raw_count_ == 0) {
                    raw_line_start_us_ = now_us;
                }
                raw_samples_[raw_count_++] = ir;
            }
        }

        if (now_us - diag_window_start_us_ < 1000000) {
            return;
        }
        ESP_LOGI(TAG,
                 "HRDIAG ir_min=%lu ir_max=%lu baseline=%ld ac_min=%ld ac_max=%ld peak_thr=%ld "
                 "samples=%lu beats=%u/4 finger=%d",
                 static_cast<unsigned long>(diag_ir_min_), static_cast<unsigned long>(diag_ir_max_),
                 static_cast<long>(baseline), static_cast<long>(diag_has_ac_ ? diag_ac_min_ : 0),
                 static_cast<long>(diag_has_ac_ ? diag_ac_max_ : 0),
                 static_cast<long>(peak_threshold), static_cast<unsigned long>(diag_samples_),
                 static_cast<unsigned>(beats), finger ? 1 : 0);
        diag_window_start_us_ = 0;

        if (raw_count_ > 0) {
            int len = snprintf(raw_line_, sizeof(raw_line_), "HRRAW,%llu",
                               static_cast<unsigned long long>(
                                   (raw_line_start_us_ - raw_start_us_) / 1000));
            for (size_t i = 0; i < raw_count_ && len > 0 &&
                               len < static_cast<int>(sizeof(raw_line_));
                 ++i) {
                len += snprintf(raw_line_ + len, sizeof(raw_line_) - len, ",%lu",
                                static_cast<unsigned long>(raw_samples_[i]));
            }
            ESP_LOGI(TAG, "%s", raw_line_);
            raw_count_ = 0;
        }
    }

    // Resets FIFO_WR_PTR, OVF_COUNTER and FIFO_RD_PTR so the stream restarts from empty.
    void ClearFifo() {
        if (!WriteRegister(kRegFifoWritePtr, 0) || !WriteRegister(kRegOvfCounter, 0) ||
            !WriteRegister(kRegFifoReadPtr, 0)) {
            ++alive_i2c_errors_;
        }
    }

    static uint32_t Field18(const uint8_t* bytes) {
        return ((static_cast<uint32_t>(bytes[0]) << 16) | (static_cast<uint32_t>(bytes[1]) << 8) |
                bytes[2]) &
               0x3FFFF;
    }

    // 18-bit data leaves bits 23:18 zero, so the first byte of each 3-byte field must be <= 0x03.
    static bool SampleValid(const uint8_t* sample) {
        return sample[0] <= 0x03 && sample[3] <= 0x03;
    }

    // DIAGNOSTIC, REMOVE LATER: counts a sample, keeps its raw bytes for HRALIVE, logs the first
    // 12 as HRSAMPLE. Returns whether its bytes are valid 18-bit data.
    bool RecordSample(const uint8_t* sample) {
        memcpy(alive_raw_, sample, sizeof(alive_raw_));
        ++alive_samples_;
        ++samples_total_;
        const bool valid = SampleValid(sample);
        if (!valid) {
            ++alive_invalid_;
            ++invalid_total_;
        }
        if (samples_logged_ < kSampleLogCount) {
            ++samples_logged_;
            ESP_LOGI(TAG, "HRSAMPLE i=%d bytes=%02X %02X %02X %02X %02X %02X%s", samples_logged_,
                     sample[0], sample[1], sample[2], sample[3], sample[4], sample[5],
                     valid ? "" : " INVALID");
        }
        return valid;
    }

    // Reads FIFO_WR_PTR, OVF_COUNTER and FIFO_RD_PTR with one single-register read each.
    bool ReadFifoState(uint8_t* write_ptr, uint8_t* overflow, uint8_t* read_ptr) {
        if (!ReadRegister(kRegFifoWritePtr, write_ptr, 1) ||
            !ReadRegister(kRegOvfCounter, overflow, 1) ||
            !ReadRegister(kRegFifoReadPtr, read_ptr, 1)) {
            ++alive_i2c_errors_;
            return false;
        }
        *write_ptr &= 0x1F;
        *overflow &= 0x1F;
        *read_ptr &= 0x1F;
        return true;
    }

    void Run() {
        alive_start_us_ = esp_timer_get_time();
        if (kHrObserve) {
            RunObserve();  // never returns
        }
        // Init() cleared the FIFO, but this low-priority task may first run seconds later. With
        // rollover off, 32 entries (1.28 s) fill it, after which FIFO_WR_PTR == FIFO_RD_PTR, so
        // start from a clean FIFO here. PollFifo() also recovers from any later overflow.
        ClearFifo();
        while (true) {
            PollFifo();
            LogAlive();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // DIAGNOSTIC, REMOVE LATER: observation mode (kHrObserve). Never drains or clears the FIFO
    // because of an overflow.
    //  1. HROBS: 40 polls 100 ms apart, logging pointers, OVF_COUNTER and both interrupt status
    //     registers (single-register reads), plus the same three registers read as one 3-byte
    //     burst from 0x04, to compare. No samples are read, so the FIFO is seen filling up.
    //  2. 5 s of one-sample-per-poll reads (only when (wr - rd) & 0x1F != 0), validating every
    //     sample and logging the first 12 as HRSAMPLE.
    //  3. Normal operation on that same read path, with HRALIVE every 2 s.
    // The FIFO is cleared once before step 1 and once before step 2 (phase starts, not
    // overflow recovery): otherwise both steps would start from the FIFO left full by step 0.
    void RunObserve() {
        ESP_LOGI(TAG, "HROBS start: sensor task began %lld ms after Init(); clearing FIFO once",
                 static_cast<long long>((esp_timer_get_time() - init_done_us_) / 1000));
        ClearFifo();
        for (int n = 1; n <= kObservePolls; ++n) {
            vTaskDelay(pdMS_TO_TICKS(kObservePollMs));
            uint8_t write_ptr = 0;
            uint8_t overflow = 0;
            uint8_t read_ptr = 0;
            uint8_t int1 = 0;
            uint8_t int2 = 0;
            uint8_t burst[3] = {};
            const bool state_ok = ReadFifoState(&write_ptr, &overflow, &read_ptr);
            const bool int_ok = ReadRegister(0x00, &int1, 1) && ReadRegister(0x01, &int2, 1);
            const bool burst_ok = ReadRegister(kRegFifoWritePtr, burst, sizeof(burst));
            ESP_LOGI(TAG,
                     "HROBS n=%d wr=%u rd=%u ovf=%u int1=0x%02X int2=0x%02X "
                     "burst(04,05,06)=%02X/%02X/%02X%s",
                     n, write_ptr, read_ptr, overflow, int1, int2, burst[0], burst[1], burst[2],
                     state_ok && int_ok && burst_ok ? "" : " i2c_err");
        }

        ESP_LOGI(TAG, "HROBS done; clearing FIFO once, then 5 s of validated one-sample reads");
        ClearFifo();
        alive_start_us_ = esp_timer_get_time();
        const int64_t check_end_us = esp_timer_get_time() + kSampleCheckUs;
        while (esp_timer_get_time() < check_end_us) {
            PollOneSample();
            LogAlive();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        ESP_LOGI(TAG, "HRSAMPLE check done: %lu invalid of %lu samples in 5 s",
                 static_cast<unsigned long>(invalid_total_),
                 static_cast<unsigned long>(samples_total_));

        while (true) {
            PollOneSample();
            LogAlive();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // Observation read path: at most one sample per poll, no overflow recovery. Invalid samples
    // are counted and not passed to beat detection.
    void PollOneSample() {
        ++alive_polls_;
        uint8_t write_ptr = 0;
        uint8_t overflow = 0;
        uint8_t read_ptr = 0;
        if (!ReadFifoState(&write_ptr, &overflow, &read_ptr)) {
            return;
        }
        alive_write_ptr_ = write_ptr;
        alive_read_ptr_ = read_ptr;
        alive_ovf_last_ = overflow;
        if (overflow != 0) {
            ++alive_ovf_polls_;
        }
        if (((write_ptr - read_ptr) & 0x1F) == 0) {
            return;
        }
        ++alive_data_polls_;
        uint8_t sample[6];
        if (!ReadRegister(kRegFifoData, sample, sizeof(sample))) {
            ++alive_i2c_errors_;
            return;
        }
        if (!RecordSample(sample)) {
            return;
        }
        UpdateBeat(Field18(sample + 3), esp_timer_get_time());
    }

    // Recovery read path (kHrObserve == false): reads every available sample. A non-zero
    // OVF_COUNTER is taken to mean a full FIFO (FIFO_WR_PTR == FIFO_RD_PTR), which is drained and
    // cleared. Still uses a 3-byte burst read of 0x04-0x06.
    void PollFifo() {
        ++alive_polls_;
        uint8_t regs[3] = {};  // 0x04 FIFO_WR_PTR, 0x05 OVF_COUNTER, 0x06 FIFO_RD_PTR
        if (!ReadRegister(kRegFifoWritePtr, regs, sizeof(regs))) {
            ++alive_i2c_errors_;
            return;
        }
        const uint8_t write_ptr = regs[0] & 0x1F;
        const uint8_t overflow = regs[1] & 0x1F;
        const uint8_t read_ptr = regs[2] & 0x1F;
        alive_write_ptr_ = write_ptr;
        alive_read_ptr_ = read_ptr;
        alive_ovf_last_ = overflow;
        const bool full = overflow != 0;
        if (full) {
            ++alive_ovf_polls_;
        }
        const size_t available = full ? kFifoDepth : ((write_ptr - read_ptr) & 0x1F);
        if (available == 0) {
            return;
        }
        ++alive_data_polls_;

        const uint64_t now_us = esp_timer_get_time();
        for (size_t i = 0; i < available; ++i) {
            uint8_t sample[6];
            if (!ReadRegister(kRegFifoData, sample, sizeof(sample))) {
                ++alive_i2c_errors_;
                break;
            }
            RecordSample(sample);
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
            ClearFifo();
            ++alive_recoveries_;
            ++recoveries_total_;
        }
    }

    // DIAGNOSTIC, REMOVE LATER: every 2 s, one HRALIVE line showing whether samples flow:
    // polls, polls with data, samples, invalid samples, polls that saw an overflow, recoveries,
    // the last RED/IR (masked and raw 3-byte fields), FIFO pointers, registers read back and
    // I2C errors in the interval.
    void LogAlive() {
        const uint64_t now_us = esp_timer_get_time();
        if (now_us - alive_start_us_ < kAliveIntervalUs) {
            return;
        }
        static constexpr uint8_t kRegs[] = {0x05, 0x08, 0x09, 0x0A, 0x0C, 0x0D, 0xFF};
        static constexpr const char* kNames[] = {"ovf", "r08", "r09", "r0A", "r0C", "r0D", "id"};
        int len = snprintf(alive_line_, sizeof(alive_line_),
                           "HRALIVE polls=%lu data_polls=%lu samples=%lu invalid=%lu "
                           "ovf_polls=%lu ovf_last=%u recoveries=%lu recoveries_total=%lu "
                           "ir=%lu red=%lu ir_raw=%02X%02X%02X red_raw=%02X%02X%02X wr=%u rd=%u",
                           static_cast<unsigned long>(alive_polls_),
                           static_cast<unsigned long>(alive_data_polls_),
                           static_cast<unsigned long>(alive_samples_),
                           static_cast<unsigned long>(alive_invalid_),
                           static_cast<unsigned long>(alive_ovf_polls_), alive_ovf_last_,
                           static_cast<unsigned long>(alive_recoveries_),
                           static_cast<unsigned long>(recoveries_total_),
                           static_cast<unsigned long>(Field18(alive_raw_ + 3)),
                           static_cast<unsigned long>(Field18(alive_raw_)), alive_raw_[3],
                           alive_raw_[4], alive_raw_[5], alive_raw_[0], alive_raw_[1],
                           alive_raw_[2], alive_write_ptr_, alive_read_ptr_);
        for (size_t i = 0; i < sizeof(kRegs) && len > 0 && len < static_cast<int>(sizeof(alive_line_));
             ++i) {
            uint8_t value = 0;
            if (ReadRegister(kRegs[i], &value, 1)) {
                len += snprintf(alive_line_ + len, sizeof(alive_line_) - len, " %s=0x%02X",
                                kNames[i], value);
            } else {
                ++alive_i2c_errors_;
                len += snprintf(alive_line_ + len, sizeof(alive_line_) - len, " %s=ERR",
                                kNames[i]);
            }
        }
        if (len > 0 && len < static_cast<int>(sizeof(alive_line_))) {
            snprintf(alive_line_ + len, sizeof(alive_line_) - len, " i2c_err=%lu",
                     static_cast<unsigned long>(alive_i2c_errors_));
        }
        ESP_LOGI(TAG, "%s", alive_line_);
        alive_start_us_ = now_us;
        alive_samples_ = 0;
        alive_i2c_errors_ = 0;
        alive_polls_ = 0;
        alive_data_polls_ = 0;
        alive_invalid_ = 0;
        alive_ovf_polls_ = 0;
        alive_recoveries_ = 0;
    }

    static constexpr gpio_num_t kSdaPin = GPIO_NUM_8;
    static constexpr gpio_num_t kSclPin = GPIO_NUM_9;
    static constexpr uint32_t kI2cSpeedHz = 100000;
    static constexpr int kI2cTimeoutMs = 20;
    static constexpr int kBootInitAttempts = 3;
    static constexpr uint64_t kInitRetryUs = 2000000;
    static constexpr uint32_t kTaskStackBytes = 5120;

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

    // DIAGNOSTIC, REMOVE LATER: line levels and an address scan after a failed init attempt.
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
        ESP_LOGW(TAG, "MAX30102 diag: SDA(GPIO8)=%d SCL(GPIO9)=%d with internal pull-ups", sda, scl);
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
            init_error_ = "Failed to initialize MAX30102 I2C bus 2: " +
                          std::string(esp_err_to_name(err));
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
            // DIAGNOSTIC, REMOVE LATER: something ACKs at 0x57 but is not a MAX30102.
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
            !WriteRegister(0x0A, 0x27) || !WriteRegister(0x0C, kLedCurrent) ||
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
        init_done_us_ = esp_timer_get_time();  // set before the sensor task can read it
        if (task_ == nullptr &&
            xTaskCreate(SensorTask, "heart_rate", kTaskStackBytes, this, 1, &task_) != pdPASS) {
            task_ = nullptr;
            init_error_ = "Failed to create MAX30102 sensor task";
            ReleaseI2c();
            return false;
        }
        init_error_.clear();
        ready_ = true;
        ESP_LOGI(TAG, "MAX30102 found at 0x57 (I2C %lu Hz); sensor task stack=%lu bytes, priority=1",
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
            ESP_LOGE(TAG, "MAX30102 unavailable after %d attempts (%lld ms); the tool retries later",
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
                return std::unexpected(
                    std::string("Heart-rate sensor initialization is in progress; ask again shortly."));
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
        const bool fresh = latest_bpm_ > 0 && last_beat_us_ != 0 &&
                           now_us - last_beat_us_ <= kHeartRateStaleUs;
        const int bpm = latest_bpm_;
        const size_t beats = interval_count_;
        const uint32_t last_ir = last_ir_;
        const uint32_t samples = total_samples_;
        xSemaphoreGive(mutex_);

        if (fresh) {
            return "BPM: " + std::to_string(bpm) + ". This is a reference value, not medical.";
        }
        const char* finger_state = samples == 0              ? "no sensor data yet"
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

// DIAGNOSTIC, REMOVE LATER: wake-word SRAM measurement only.
// Logs the internal-SRAM low-water mark so wake-word memory cost can be compared with the
// ~48 KB baseline. Led::OnStateChanged() is the board's only state-change hook. The audio
// engine starts on the first switch to idle, so the first report fires 5 s later; each later
// idle (end of a conversation) reports again after 20 s.
class MemoryReportingLed : public SingleLed {
private:
    esp_timer_handle_t timer_ = nullptr;
    std::atomic<bool> engine_reported_{false};  // written by esp_timer, read on main task

    static void Report(void* arg) {
        auto* self = static_cast<MemoryReportingLed*>(arg);
        ESP_LOGI(TAG, "Internal SRAM (%s): min free=%u bytes, free now=%u bytes",
                 self->engine_reported_ ? "20 s after conversation end" : "after audio engine start",
                 static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)),
                 static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
        self->engine_reported_ = true;
    }

public:
    explicit MemoryReportingLed(gpio_num_t gpio) : SingleLed(gpio) {
        esp_timer_create_args_t args = {
            .callback = Report,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "robot_sram_log",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&args, &timer_) != ESP_OK) {
            timer_ = nullptr;
        }
    }

    void OnStateChanged() override {
        SingleLed::OnStateChanged();
        if (timer_ == nullptr) {
            return;
        }
        esp_timer_stop(timer_);
        if (Application::GetInstance().GetDeviceState() == kDeviceStateIdle) {
            esp_timer_start_once(timer_, engine_reported_ ? 20000000 : 5000000);
        }
    }
};  // DIAGNOSTIC, REMOVE LATER: end of MemoryReportingLed

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
        // DIAGNOSTIC, REMOVE LATER: restore `static SingleLed led(BUILTIN_LED_GPIO);`.
        static MemoryReportingLed led(BUILTIN_LED_GPIO);
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

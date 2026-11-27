#include "eyes_display.h"

#include "application.h"

#include <esp_log.h>
#include <esp_random.h>

#include <cstring>

#define TAG "EyesDisplay"

namespace {
constexpr uint32_t kFramePeriodMs = 50;  // ~20 fps; a full frame is ~25 ms on 400 kHz I2C
constexpr size_t kPaletteBytes = 2 * sizeof(lv_color32_t);  // I1: 2-entry palette before pixels
}  // namespace

EyesDisplay::EyesDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                         int width, int height, bool mirror_x, bool mirror_y)
    : OledDisplay(panel_io, panel, width, height, mirror_x, mirror_y), renderer_(esp_random()) {}

void EyesDisplay::SetupUI() {
    OledDisplay::SetupUI();
    DisplayLockGuard lock(this);
    if (canvas_ != nullptr) {
        return;  // SetupUI() called again
    }
    draw_buf_ = lv_draw_buf_create(robot_emo::kEyesWidth, robot_emo::kEyesHeight,
                                   LV_COLOR_FORMAT_I1, LV_STRIDE_AUTO);
    if (draw_buf_ == nullptr) {
        ESP_LOGE(TAG, "No memory for the eye canvas; keeping the stock UI");
        return;
    }
    canvas_ = lv_canvas_create(lv_screen_active());
    lv_canvas_set_draw_buf(canvas_, draw_buf_);
    // On this OLED port, black lights a pixel and white leaves it off (esp_lvgl_port's
    // monochrome transform; the stock UI draws black text on white). Frame bit 1 = lit.
    lv_canvas_set_palette(canvas_, 0, lv_color32_make(0xFF, 0xFF, 0xFF, 0xFF));
    lv_canvas_set_palette(canvas_, 1, lv_color32_make(0, 0, 0, 0xFF));
    memset(draw_buf_->data + kPaletteBytes, 0, draw_buf_->header.stride * robot_emo::kEyesHeight);
    lv_obj_set_pos(canvas_, 0, 0);
    lv_obj_add_flag(canvas_, LV_OBJ_FLAG_HIDDEN);  // Tick() shows it when allowed
    lv_obj_move_foreground(canvas_);
    timer_ = lv_timer_create(OnTimer, kFramePeriodMs, this);
    ESP_LOGI(TAG, "Animated eyes ready (%d ms per frame)", static_cast<int>(kFramePeriodMs));
}

bool EyesDisplay::SetEmotion(const char* emotion) {
    expression_ = static_cast<int>(robot_emo::ExpressionFromEmotion(emotion));
    return OledDisplay::SetEmotion(emotion);  // keeps the stock UI current underneath
}

void EyesDisplay::SetChatMessage(const char* role, const char* content) {
    if (role != nullptr && strcmp(role, "system") == 0) {
        system_message_active_ = content != nullptr && content[0] != '\0';
    }
    OledDisplay::SetChatMessage(role, content);
}

void EyesDisplay::OnTimer(lv_timer_t* timer) {
    static_cast<EyesDisplay*>(lv_timer_get_user_data(timer))->Tick();
}

// Eyes only in normal conversation states and when no system message (alert, error, OTA,
// activation text) needs to be read; otherwise the stock UI underneath shows through.
bool EyesDisplay::ShouldShowEyes() const {
    if (system_message_active_) {
        return false;
    }
    switch (Application::GetInstance().GetDeviceState()) {
        case kDeviceStateIdle:
        case kDeviceStateConnecting:
        case kDeviceStateListening:
        case kDeviceStateSpeaking:
            return true;
        default:
            return false;
    }
}

// While the eyes cover the screen, hide the stock UI's top-level objects. Otherwise a long
// chat message keeps scrolling underneath and forces a full render and I2C flush every
// refresh. Only objects that were visible are hidden, and only those are restored.
void EyesDisplay::HideStockUi() {
    lv_obj_t* screen = lv_screen_active();
    const uint32_t count = lv_obj_get_child_count(screen);
    for (uint32_t i = 0; i < count; ++i) {
        lv_obj_t* child = lv_obj_get_child(screen, static_cast<int32_t>(i));
        if (child != canvas_ && !lv_obj_has_flag(child, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_add_flag(child, LV_OBJ_FLAG_HIDDEN);
            hidden_stock_ui_.push_back(child);
        }
    }
}

void EyesDisplay::RestoreStockUi() {
    for (lv_obj_t* child : hidden_stock_ui_) {
        lv_obj_remove_flag(child, LV_OBJ_FLAG_HIDDEN);
    }
    hidden_stock_ui_.clear();
}

// Runs on the LVGL task, which already holds the display lock.
void EyesDisplay::Tick() {
    const bool show = ShouldShowEyes();
    const bool just_shown = show && !eyes_visible_;
    if (show != eyes_visible_) {
        eyes_visible_ = show;
        if (show) {
            HideStockUi();
            lv_obj_remove_flag(canvas_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(canvas_);
        } else {
            lv_obj_add_flag(canvas_, LV_OBJ_FLAG_HIDDEN);
            RestoreStockUi();
        }
    }
    if (!show) {
        return;
    }

    const DeviceState state = Application::GetInstance().GetDeviceState();
    renderer_.SetMode(state == kDeviceStateListening  ? robot_emo::EyeMode::kListening
                      : state == kDeviceStateSpeaking ? robot_emo::EyeMode::kSpeaking
                                                      : robot_emo::EyeMode::kIdle);
    renderer_.SetExpression(static_cast<robot_emo::Expression>(expression_.load()));
    const bool changed = renderer_.Render(lv_tick_get(), frame_);
    if (!changed && !just_shown) {
        return;  // unchanged: nothing to send over I2C
    }

    // Pixel rows follow the 2-entry palette; honour the buffer's stride.
    const uint32_t stride = draw_buf_->header.stride;
    uint8_t* pixels = draw_buf_->data + kPaletteBytes;
    for (int y = 0; y < robot_emo::kEyesHeight; ++y) {
        memcpy(pixels + y * stride, frame_ + y * robot_emo::kEyesStride, robot_emo::kEyesStride);
    }
    lv_obj_invalidate(canvas_);
}

#pragma once

// robot-emo OLED: two animated eyes fill the screen (EyeRenderer) over the stock OledDisplay UI.
// The stock UI shows through whenever something must be read; see the visibility rule in
// docs/superpowers/specs/2026-11-25-robot-emo-animated-eyes-design.md.

#include "display/oled_display.h"
#include "eye_renderer.h"

#include <atomic>
#include <vector>

class EyesDisplay : public OledDisplay {
public:
    EyesDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width,
                int height, bool mirror_x, bool mirror_y);

    void SetupUI() override;
    bool SetEmotion(const char* emotion) override;
    void SetChatMessage(const char* role, const char* content) override;

private:
    static void OnTimer(lv_timer_t* timer);
    void Tick();
    bool ShouldShowEyes() const;
    void HideStockUi();
    void RestoreStockUi();

    robot_emo::EyeRenderer renderer_;
    lv_obj_t* canvas_ = nullptr;
    lv_draw_buf_t* draw_buf_ = nullptr;
    lv_timer_t* timer_ = nullptr;
    bool eyes_visible_ = false;               // LVGL task only
    std::vector<lv_obj_t*> hidden_stock_ui_;  // LVGL task only
    uint8_t frame_[robot_emo::kEyesFrameBytes] = {};

    // Written by any task, read by the LVGL timer.
    std::atomic<int> expression_{static_cast<int>(robot_emo::Expression::kNeutral)};
    std::atomic<bool> system_message_active_{false};
};

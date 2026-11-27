#pragma once

// Animated eyes for robot-emo's 128x64 monochrome OLED. Pure C++ (no ESP-IDF or LVGL), so it
// is host-testable: scripts/tests/test_robot_emo_eyes.py. Shapes and timings follow the
// approved mockup docs/superpowers/specs/2026-11-25-robot-emo-eyes-mockup.html.

#include <array>
#include <cstdint>
#include <random>

namespace robot_emo {

enum class Expression {
    kNeutral,
    kHappy,
    kLaughing,
    kSad,
    kCrying,
    kAngry,
    kSurprised,
    kThinking,
    kSleepy,
    kLoving,
    kWinking,
    kConfused,
    kCool,
};

enum class EyeMode { kIdle, kListening, kSpeaking };

// Maps the server's emotion names to an expression; unknown names (and null) give kNeutral.
Expression ExpressionFromEmotion(const char* emotion);

// Frame buffer: 1 bit per pixel, row stride kEyesStride bytes, MSB = leftmost pixel, 1 = lit.
constexpr int kEyesWidth = 128;
constexpr int kEyesHeight = 64;
constexpr int kEyesStride = kEyesWidth / 8;
constexpr int kEyesFrameBytes = kEyesStride * kEyesHeight;

class EyeRenderer {
public:
    explicit EyeRenderer(uint32_t seed);

    void SetExpression(Expression expression) { expression_ = expression; }
    void SetMode(EyeMode mode) { mode_ = mode; }
    // Starts a blink at now_ms (random blinks also happen on their own).
    void TriggerBlink(uint32_t now_ms);

    // Draws the frame for now_ms into `frame` (kEyesFrameBytes). Returns true when it differs
    // from the previous frame. Calling again with the same now_ms draws the same frame.
    bool Render(uint32_t now_ms, uint8_t* frame);

private:
    struct Shape {
        float w = 1, h = 1, r = 0.3f;   // size scale and corner radius (fraction of base)
        float lid_in = 0, lid_out = 0;  // angry (inner) and sad (outer) lid, fraction of height
        float happy = 0;                // how far the "smile" mask covers the eye from below
        float look_x = 0, look_y = 0;   // gaze, -1..1
        float left_close = 0;           // wink: left eye closed when > 0.5
        float small = 0;                // right eye shrink (confused, thinking)
    };

    static Shape TargetShape(Expression expression);
    void Update(uint32_t now_ms);
    void Draw(uint32_t now_ms, uint8_t* frame) const;

    std::minstd_rand rng_;
    Expression expression_ = Expression::kNeutral;
    EyeMode mode_ = EyeMode::kIdle;
    Shape current_;
    bool started_ = false;
    uint32_t last_ms_ = 0;
    uint32_t blink_until_ms_ = 0;
    uint32_t next_blink_ms_ = 0;
    uint32_t next_glance_ms_ = 0;
    float glance_x_ = 0, glance_y_ = 0;
    std::array<uint8_t, kEyesFrameBytes> previous_{};
};

}  // namespace robot_emo

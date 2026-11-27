#include "eye_renderer.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace robot_emo {
namespace {

constexpr float kPi = 3.14159265f;
constexpr float kBaseEye = 34.0f;      // eye size in pixels at scale 1
constexpr float kEasePer33Ms = 0.18f;  // fraction of the way to the target per ~30 fps frame
constexpr uint32_t kBlinkMs = 140;
constexpr float kListenScale = 1.08f;

// True when `a` is at or after `b`, robust to the 49-day millisecond wrap.
bool Reached(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b) >= 0; }

class Canvas {
public:
    explicit Canvas(uint8_t* frame) : frame_(frame) {}

    void Set(int x, int y, bool on) const {
        if (x < 0 || x >= kEyesWidth || y < 0 || y >= kEyesHeight) {
            return;
        }
        uint8_t& byte = frame_[y * kEyesStride + x / 8];
        const uint8_t bit = static_cast<uint8_t>(0x80 >> (x % 8));
        byte = on ? (byte | bit) : (byte & ~bit);
    }

    // Pixel (px, py) is inside when its centre (px + 0.5, py + 0.5) is.
    void RoundRect(float x, float y, float w, float h, float r, bool on) const {
        r = std::max(0.0f, std::min({r, w / 2, h / 2}));
        for (int py = Floor(y); py <= Ceil(y + h); ++py) {
            for (int px = Floor(x); px <= Ceil(x + w); ++px) {
                const float cx = px + 0.5f, cy = py + 0.5f;
                if (cx < x || cx > x + w || cy < y || cy > y + h) {
                    continue;
                }
                const float dx = std::max({x + r - cx, 0.0f, cx - (x + w - r)});
                const float dy = std::max({y + r - cy, 0.0f, cy - (y + h - r)});
                if (dx * dx + dy * dy <= r * r) {
                    Set(px, py, on);
                }
            }
        }
    }

    void Ellipse(float cx, float cy, float rx, float ry, bool on) const {
        if (rx <= 0 || ry <= 0) {
            return;
        }
        for (int py = Floor(cy - ry); py <= Ceil(cy + ry); ++py) {
            for (int px = Floor(cx - rx); px <= Ceil(cx + rx); ++px) {
                const float dx = (px + 0.5f - cx) / rx, dy = (py + 0.5f - cy) / ry;
                if (dx * dx + dy * dy <= 1.0f) {
                    Set(px, py, on);
                }
            }
        }
    }

    void Triangle(float x0, float y0, float x1, float y1, float x2, float y2, bool on) const {
        const float area = Edge(x0, y0, x1, y1, x2, y2);
        if (std::fabs(area) < 0.01f) {
            return;
        }
        for (int py = Floor(std::min({y0, y1, y2})); py <= Ceil(std::max({y0, y1, y2})); ++py) {
            for (int px = Floor(std::min({x0, x1, x2})); px <= Ceil(std::max({x0, x1, x2})); ++px) {
                const float cx = px + 0.5f, cy = py + 0.5f;
                const float a = Edge(x1, y1, x2, y2, cx, cy) / area;
                const float b = Edge(x2, y2, x0, y0, cx, cy) / area;
                const float c = Edge(x0, y0, x1, y1, cx, cy) / area;
                if (a >= 0 && b >= 0 && c >= 0) {
                    Set(px, py, on);
                }
            }
        }
    }

    void Heart(float cx, float cy, float s) const {
        Ellipse(cx - s * 0.5f, cy - s * 0.25f, s * 0.55f, s * 0.55f, true);
        Ellipse(cx + s * 0.5f, cy - s * 0.25f, s * 0.55f, s * 0.55f, true);
        Triangle(cx - s * 1.05f, cy - s * 0.05f, cx + s * 1.05f, cy - s * 0.05f, cx, cy + s * 0.95f,
                 true);
    }

    // rows: one string per row, '1' = lit.
    void Bitmap(int x, int y, const char* const* rows, int count) const {
        for (int row = 0; row < count; ++row) {
            for (int col = 0; rows[row][col] != '\0'; ++col) {
                if (rows[row][col] == '1') {
                    Set(x + col, y + row, true);
                }
            }
        }
    }

private:
    static int Floor(float v) { return static_cast<int>(std::floor(v)); }
    static int Ceil(float v) { return static_cast<int>(std::ceil(v)); }
    static float Edge(float ax, float ay, float bx, float by, float px, float py) {
        return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    }

    uint8_t* frame_;
};

constexpr const char* kZ[] = {"11111", "00010", "00100", "01000", "11111"};
constexpr const char* kQuestion[] = {"01110", "10001", "00001", "00010", "00100", "00000", "00100"};

}  // namespace

Expression ExpressionFromEmotion(const char* emotion) {
    if (emotion == nullptr) {
        return Expression::kNeutral;
    }
    struct Entry {
        const char* name;
        Expression expression;
    };
    static constexpr Entry kTable[] = {
        {"happy", Expression::kHappy},       {"laughing", Expression::kLaughing},
        {"funny", Expression::kLaughing},    {"silly", Expression::kLaughing},
        {"sad", Expression::kSad},           {"crying", Expression::kCrying},
        {"angry", Expression::kAngry},       {"surprised", Expression::kSurprised},
        {"shocked", Expression::kSurprised}, {"thinking", Expression::kThinking},
        {"sleepy", Expression::kSleepy},     {"loving", Expression::kLoving},
        {"kissy", Expression::kLoving},      {"winking", Expression::kWinking},
        {"confused", Expression::kConfused}, {"cool", Expression::kCool},
        {"delicious", Expression::kHappy},   {"relaxed", Expression::kHappy},
        {"confident", Expression::kHappy},   {"embarrassed", Expression::kHappy},
    };
    for (const Entry& entry : kTable) {
        if (strcmp(emotion, entry.name) == 0) {
            return entry.expression;
        }
    }
    return Expression::kNeutral;
}

EyeRenderer::EyeRenderer(uint32_t seed) : rng_(seed == 0 ? 1 : seed) {}

void EyeRenderer::TriggerBlink(uint32_t now_ms) {
    started_ = true;
    blink_until_ms_ = now_ms + kBlinkMs;
}

EyeRenderer::Shape EyeRenderer::TargetShape(Expression expression) {
    Shape s;
    switch (expression) {
        case Expression::kHappy:
            s.happy = 0.55f;
            break;
        case Expression::kLaughing:
            s.happy = 0.75f;
            s.h = 0.8f;
            break;
        case Expression::kSad:
            s.lid_out = 0.45f;
            s.look_y = 0.35f;
            s.h = 0.9f;
            break;
        case Expression::kCrying:
            s.lid_out = 0.5f;
            s.look_y = 0.3f;
            break;
        case Expression::kAngry:
            s.lid_in = 0.5f;
            s.h = 0.85f;
            break;
        case Expression::kSurprised:
            s.w = 1.15f;
            s.h = 1.2f;
            s.r = 0.5f;
            break;
        case Expression::kThinking:
            s.look_x = 0.7f;
            s.look_y = -0.6f;
            s.small = 0.15f;
            break;
        case Expression::kSleepy:
            s.h = 0.22f;
            s.look_y = 0.4f;
            break;
        case Expression::kLoving:
            s.happy = 0.35f;
            break;
        case Expression::kWinking:
            s.left_close = 1;
            s.happy = 0.5f;
            break;
        case Expression::kConfused:
            s.small = 0.35f;
            s.look_x = -0.3f;
            break;
        case Expression::kCool:
            s.h = 0.55f;
            s.lid_in = 0.1f;
            break;
        case Expression::kNeutral:
            break;
    }
    return s;
}

void EyeRenderer::Update(uint32_t now_ms) {
    if (!started_) {
        started_ = true;
        next_blink_ms_ = now_ms + 1500;
        next_glance_ms_ = now_ms + 2500;
    }
    const uint32_t dt = now_ms - last_ms_;
    const bool advanced = last_ms_ == 0 || (dt > 0 && dt < 0x80000000u);
    last_ms_ = now_ms;
    if (!advanced) {
        return;  // same instant (or time went backwards): keep the state, redraw the same frame
    }

    const Shape target = TargetShape(expression_);
    const float k = std::min(1.0f, kEasePer33Ms * std::min<uint32_t>(dt, 200) / 33.0f);
    auto ease = [k](float& value, float goal) { value += (goal - value) * k; };
    ease(current_.w, target.w);
    ease(current_.h, target.h);
    ease(current_.r, target.r);
    ease(current_.lid_in, target.lid_in);
    ease(current_.lid_out, target.lid_out);
    ease(current_.happy, target.happy);
    ease(current_.look_x, target.look_x);
    ease(current_.look_y, target.look_y);
    ease(current_.left_close, target.left_close);
    ease(current_.small, target.small);

    if (Reached(now_ms, next_blink_ms_)) {
        if (expression_ != Expression::kSleepy) {
            blink_until_ms_ = now_ms + kBlinkMs;
        }
        next_blink_ms_ = now_ms + 1800 + rng_() % 3500;
    }
    if (mode_ != EyeMode::kIdle) {
        glance_x_ = glance_y_ = 0;  // listening and speaking look straight at the user
    } else if (Reached(now_ms, next_glance_ms_)) {
        auto unit = [this]() { return static_cast<float>(rng_() % 1000) / 1000.0f - 0.5f; };
        if (rng_() % 10 < 7) {
            glance_x_ = unit() * 0.9f;
            glance_y_ = unit() * 0.5f;
        } else {
            glance_x_ = glance_y_ = 0;
        }
        next_glance_ms_ = now_ms + 1500 + rng_() % 2500;
    }
}

void EyeRenderer::Draw(uint32_t now_ms, uint8_t* frame) const {
    memset(frame, 0, kEyesFrameBytes);
    const Canvas canvas(frame);
    // Animation phase in ms, wrapped hourly so float precision stays sub-millisecond after days
    // of uptime (a raw float of now_ms steps by tens of ms after a few days).
    const float t = static_cast<float>(now_ms % 3600000u);
    const bool blinking = !Reached(now_ms, blink_until_ms_);
    const float scale = mode_ == EyeMode::kListening ? kListenScale : 1.0f;
    const float bounce = mode_ == EyeMode::kSpeaking ? std::fabs(std::sin(t / 110.0f)) * 2.0f : 0;
    const float base = kBaseEye;
    const float look_x = (current_.look_x + glance_x_) * base * 0.35f;
    const float look_y = (current_.look_y + glance_y_) * base * 0.25f - bounce;
    const float gap = base * 0.55f;

    for (int side : {-1, 1}) {
        const float shrink = side == 1 ? 1.0f - current_.small : 1.0f;
        const float w = base * current_.w * shrink * scale;
        const float h = base * current_.h * shrink * scale;
        const float cx = kEyesWidth / 2.0f + side * (gap / 2 + base / 2) + look_x;
        const float cy = kEyesHeight / 2.0f + look_y;

        if (expression_ == Expression::kLoving) {
            canvas.Heart(cx, cy, base * 0.42f * (1 + std::sin(t / 180.0f) * 0.08f));
            continue;
        }
        if (blinking || (side == -1 && current_.left_close > 0.5f)) {
            canvas.RoundRect(cx - w / 2, cy - 1.5f, w, 3, 1.5f, true);
            continue;
        }
        canvas.RoundRect(cx - w / 2, cy - h / 2, w, h, base * current_.r, true);
        if (current_.happy > 0.02f) {
            canvas.Ellipse(cx, cy + h * (1.05f - current_.happy), w * 0.75f, h * 0.62f, false);
        }
        const float inner = static_cast<float>(-side);  // direction towards the nose
        const float top = cy - h / 2 - 2;
        const float edge_in = cx + inner * (w / 2 + 2), edge_out = cx - inner * (w / 2 + 2);
        if (current_.lid_in * h > 0.5f) {  // angry: inner corner pulled down
            canvas.Triangle(edge_in, cy - h / 2 + current_.lid_in * h, edge_out, top, edge_in, top,
                            false);
        }
        if (current_.lid_out * h > 0.5f) {  // sad: outer corner drooping
            canvas.Triangle(edge_out, cy - h / 2 + current_.lid_out * h, edge_in, top, edge_out,
                            top, false);
        }
        if (expression_ == Expression::kCool) {  // sunglasses bar
            canvas.RoundRect(cx - w / 2 - 3, cy - h / 2 - 2, w + 6, 3, 0, true);
        }
        if (expression_ == Expression::kCrying && side == 1) {
            const float fall = std::fmod(t / 12.0f, kEyesHeight * 0.6f);
            canvas.Ellipse(cx + w * 0.3f, cy + h / 2 + 2 + fall, 1.6f, 2.6f, true);
        }
    }

    if (expression_ == Expression::kSleepy) {
        const int drift = static_cast<int>(std::fmod(t / 40.0f, 20.0f));
        canvas.Bitmap(kEyesWidth - 16, 10 - drift / 3, kZ, 5);
        canvas.Bitmap(kEyesWidth - 9, 4 - drift / 4, kZ, 5);
    } else if (expression_ == Expression::kConfused) {
        canvas.Bitmap(kEyesWidth - 10, 3, kQuestion, 7);
    } else if (expression_ == Expression::kThinking) {
        const int dots = static_cast<int>(now_ms / 350 % 4);
        for (int i = 0; i < dots && i < 3; ++i) {
            canvas.RoundRect(kEyesWidth - 18 + i * 5, 4, 2, 2, 0, true);
        }
    }
}

bool EyeRenderer::Render(uint32_t now_ms, uint8_t* frame) {
    Update(now_ms);
    Draw(now_ms, frame);
    const bool changed = memcmp(frame, previous_.data(), kEyesFrameBytes) != 0;
    if (changed) {
        memcpy(previous_.data(), frame, kEyesFrameBytes);
    }
    return changed;
}

}  // namespace robot_emo

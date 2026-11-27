# robot-emo Animated Eyes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Two large animated eyes fill the robot's 128x64 OLED: they blink, glance, change shape with the server's emotion and react to listening and speaking. The stock text UI comes back whenever something must be read.

**Architecture:**
- A pure C++ `EyeRenderer` draws one frame into a 1-bit 128x64 buffer. It holds the emotion targets, easing, blinks and glances, and is host-testable.
- `EyesDisplay`, a board subclass of `OledDisplay`, puts an LVGL I1 canvas over the stock UI and runs the renderer from an `lv_timer`. It shows or hides the canvas according to the spec's visibility rule.

**Tech Stack:** ESP-IDF v6.0.3, LVGL 9.5 (`CONFIG_LV_USE_CANVAS=y`, `CONFIG_LV_DRAW_SW_SUPPORT_I1=y`), C++23, Python `unittest` with a host `c++` for the renderer test.

**Spec:** `docs/superpowers/specs/2026-11-25-robot-emo-animated-eyes-design.md`. Drawing reference: `docs/superpowers/specs/2026-11-25-robot-emo-eyes-mockup.html` (its JS `target()`, `eyes()` and `update()` are the shapes and timings the owner approved).

## Global Constraints

- Board-only: new files in `main/boards/robot-emo/` and edits to `robot_emo_board.cc` and its README. No edits to `main/display/*`, `main/application.*` or other upstream files.
- Owner's CLAUDE.md: **do not commit unless the owner asks**. The plan's commit steps are replaced by "leave in the working tree".
- Visibility: eyes hidden when the device state is not idle, connecting, listening or speaking, OR a non-empty system message is active. `ShowNotification` alone never hides them. When the rules disagree, hiding wins.
- Listening: size x1.08, look centred, no glancing. Speaking: a vertical bounce added to any emotion. Neither replaces the emotion.
- Unknown emotion names, including `robot_2` and null, give neutral.
- Drawing: about 20 fps. Invalidate only when the frame changed (OLED is I2C at 400 kHz, about 25 ms per full frame).
- Keep internal SRAM near today's minimum of ~37 KB. The frame buffer is about 1 KB.

## Review Focus

- Activation code or Wi-Fi setup text must be readable at boot. The canvas must stay hidden in starting, wifi-configuring and activating states. Checked on hardware.
- `Alert()` during idle (e.g. a server-connection error) must show its text until it is cleared, not be covered by the eyes. Checked on hardware by turning the router off.
- A late `SetEmotion` from a non-LVGL task while the timer is drawing must not race. Emotion and system-message state are atomics read by the timer. Covered by design: no shared non-atomic state across tasks.
- Drawing at the screen edges (glance plus large surprised eyes) must not write outside the 1 KB buffer. Test: `stays_in_bounds`.
- A frame that did not change must not be flushed, so I2C is not saturated while idle and still. Test: `unchanged_frame_not_dirty`.

---

### Task 1: EyeRenderer (pure, host-tested)

**Files:**
- Create: `main/boards/robot-emo/eye_renderer.h`, `main/boards/robot-emo/eye_renderer.cc`
- Test: `scripts/tests/test_robot_emo_eyes.py`. Same pattern as `test_robot_emo_weather.py`: `CXX` or skip, compile `eye_renderer.cc` with a driver whose `main()` runs a named case.

**Interfaces (namespace `robot_emo`):**
- `enum class Expression { kNeutral, kHappy, kLaughing, kSad, kCrying, kAngry, kSurprised, kThinking, kSleepy, kLoving, kWinking, kConfused, kCool };`
- `enum class EyeMode { kIdle, kListening, kSpeaking };`
- `Expression ExpressionFromEmotion(const char* emotion);` maps the 21 names:
  - funny and silly to kLaughing
  - shocked to kSurprised
  - kissy to kLoving
  - delicious, relaxed, confident and embarrassed to kHappy
  - each name with its own enum value to that value
  - anything else, including null and "robot_2", to kNeutral
- `constexpr int kEyesWidth = 128, kEyesHeight = 64, kEyesStride = 16;` The frame buffer is `kEyesStride * kEyesHeight` = 1024 bytes, 1 bit per pixel, MSB = leftmost pixel, 1 = lit.
- `class EyeRenderer { public: explicit EyeRenderer(uint32_t seed); void SetExpression(Expression e); void SetMode(EyeMode m); void TriggerBlink(uint32_t now_ms); bool Render(uint32_t now_ms, uint8_t* frame); };`
  - `Render` clears and draws the frame.
  - It returns true when the frame differs from the previous `Render` output; it keeps its own 1 KB copy for the comparison.
  - Random blinks and glances use `std::minstd_rand` seeded by `seed`.

- [ ] **Step 1: Write the failing test.** Cases. "Lit" means the count of set bits; "converge" means 60 frames 50 ms apart.
  - `maps_emotions`: `funny→kLaughing`, `shocked→kSurprised`, `kissy→kLoving`, `sleepy→kSleepy`, `robot_2→kNeutral`, `nullptr→kNeutral`.
  - `draws_two_eyes`: neutral, converged. Lit is between 600 and 3000, and both halves (x<64, x>=64) are lit.
  - `blink_closes_eyes`: neutral, converged, then `TriggerBlink(t)`. `Render(t+20)` lit < 25% of the converged lit.
  - `happy_lights_less_low`: lit pixels in rows 40-63, happy converged < neutral converged.
  - `stays_in_bounds`: a 1024-byte frame inside a buffer with 64 canary bytes on each side. Every expression × every mode, 200 frames each, includes surprised plus listening. The canaries are unchanged.
  - `unchanged_frame_not_dirty`: listening, neutral, converged. Then `Render(t)` and `Render(t)` again at the same `t`; the second returns false.
- [ ] **Step 2: Run** `python -m unittest scripts.tests.test_robot_emo_eyes -v` with `PYTHONUTF8=1`. Expected: a compile failure because `eye_renderer.h` is missing, or SKIP with no host compiler. On SKIP, note it and rely on the firmware build for compile coverage.
- [ ] **Step 3: Implement** `eye_renderer.{h,cc}`. Port the mockup's model:
  - Parameters per expression: w, h, r, lidIn, lidOut, happy, lookX, lookY, leftClose, small.
  - Easing 0.18 per frame; blink 140 ms every 1.8-5.3 s; glance every 1.5-4 s in idle only.
  - Base eye 34 px; listening x1.08 and look centred; speaking bounce `|sin(t/110)|*2` px.
  - Extras: hearts (loving), a falling tear (crying), "z" as a 5x5 bitmap (sleepy), "?" as a 5x7 bitmap (confused), 3 dots (thinking), a sunglasses bar (cool).
  - Raster helpers on the 1-bit buffer, with every pixel write clipped: filled rounded rect, filled ellipse (lit or clear), filled triangle (clear), heart from two circles and a triangle.
- [ ] **Step 4: Run the test again.** Expected: OK, or SKIP. Then run the firmware build to compile `eye_renderer.cc` (command in Task 2 Step 5). Expected: `Project build complete`, no warnings from the file.
- [ ] **Step 5: Leave in the working tree.** No commit (owner's rule).

### Task 2: EyesDisplay and board wiring

**Files:**
- Create: `main/boards/robot-emo/eyes_display.h`, `main/boards/robot-emo/eyes_display.cc`
- Modify: `main/boards/robot-emo/robot_emo_board.cc` at the `new (std::nothrow) OledDisplay(...)` call (~line 951): create `EyesDisplay` with the same arguments.
- Modify: `main/boards/robot-emo/README.md`: a "Display: animated eyes" section covering the behaviour, the visibility rule, the expression mapping and the hardware checks.

**Interfaces:**
- Consumes: `robot_emo::EyeRenderer`, `ExpressionFromEmotion`, `EyeMode`, `kEyes*` from Task 1.
- Produces: `class EyesDisplay : public OledDisplay`, with the same constructor as `OledDisplay`. Overrides:
  - `SetupUI()`: base, then the canvas and the timer.
  - `SetEmotion(const char*)`: base, then store the expression in a `std::atomic<int>`. Returns the base's result.
  - `SetChatMessage(const char* role, const char* content)`: base. When the role is "system", set `std::atomic<bool> system_message_active_ = content && *content`.

- [ ] **Step 1: Write `EyesDisplay`.**
  - In `SetupUI()`, after `OledDisplay::SetupUI()`, under `DisplayLockGuard`:
    - create `lv_canvas` on `lv_screen_active()`, size 128x64, position 0,0, `lv_obj_move_foreground`;
    - buffer `LV_CANVAS_BUF_SIZE(128, 64, 1, LV_DRAW_BUF_STRIDE_ALIGN)` (static or member, internal RAM), set with `lv_canvas_set_buffer(..., LV_COLOR_FORMAT_I1)`;
    - palette index 0 black, index 1 white (`lv_canvas_set_palette`);
    - `lv_timer_create(cb, 50, this)`.
  - The timer callback runs on the LVGL task, which already holds the lock:
    - Compute visible as state in {idle, connecting, listening, speaking} (`Application::GetInstance().GetDeviceState()`) and `!system_message_active_`. Toggle `LV_OBJ_FLAG_HIDDEN` only when it changes.
    - If hidden, return.
    - Set the mode from the state: listening → kListening, speaking → kSpeaking, else kIdle. Set the expression from the atomic.
    - `Render(lv_tick_get(), scratch)`. If it returns true, copy the frame into the canvas pixel area (after the 8-byte palette, honouring the canvas stride) and `lv_obj_invalidate(canvas)`.
  - Seed the renderer from `esp_random()`.
- [ ] **Step 2: Wire the board.** Replace the `OledDisplay` construction with `EyesDisplay`. Keep the `NoDisplay` fallback path unchanged.
- [ ] **Step 3: README section** (see Files).
- [ ] **Step 4: Format** only the new files and the changed lines (`clang-format -i --lines=...` for `robot_emo_board.cc`).
- [ ] **Step 5: Build.** Activate with `. C:\Espressif\tools\Microsoft.v6.0.3.PowerShell_profile.ps1`, then run `python scripts\build.py robot-emo --name robot-emo --language vi-VN` with `PYTHONUTF8=1`. Expected: `Project build complete`, no warnings from the robot-emo files.
- [ ] **Step 6: Hardware check (owner)**, from the spec's Validation:
  - boot and activation text are readable;
  - idle eyes blink and glance;
  - the listening and speaking cues work, and emotions follow the replies;
  - Wi-Fi provisioning text still shows;
  - an `Alert` shows text;
  - minimal SRAM stays near 37 KB;
  - no audio glitches.
- [ ] **Step 7: Leave in the working tree** and report. No commit.

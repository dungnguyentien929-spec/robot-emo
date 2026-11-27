# robot-emo: animated eyes on the OLED

Status: draft for approval, 2026-11-25 (builder-checked). Size: project (one board, about 300-500 lines).

## Intent

The owner finds the robot's 128x64 OLED "boring, emotionless, not lively". Today it shows
a status bar, a small static 30 px emoji glyph for the emotion, and scrolling chat text.
The owner wants the robot to feel alive: two large animated eyes that fill the screen,
blink, glance around and change shape with the conversation's emotion. The owner chose
this look ("A - Toàn mắt") from an animated browser mockup
(`oled-mockup.html`, shown during brainstorming). The panel is a blue monochrome SSD1306,
so colour is fixed by the hardware.

## Goals

- Two large eyes fill the whole 128x64 screen during normal use (idle, connecting,
  listening, speaking). No chat text on screen.
- **Alive when idle:** blinks every 2-5 s at random intervals, glances in random directions,
  and moves smoothly between shapes.
- **Emotion:** each of the 21 emotion names the server can send gets a distinct eye
  expression. Changes animate (interpolated), never jump.
- **State cues:**
  - Listening: eyes slightly larger and centred (looking at the user), no glancing.
  - Speaking: a gentle up-down bounce in rhythm.
- Setup and error screens still work. While the device is not in normal conversation
  (Wi-Fi provisioning, activation code, OTA upgrade, errors, startup), and while a system
  message (alert, error, OTA text) is active, the stock OLED UI is shown instead of the eyes, so
  activation codes and Wi-Fi instructions remain readable.

## Non-goals

- Showing chat text, clock or weather on screen (options B/C were not chosen).
- Sound effects, motor movement or LED changes tied to emotions.
- Any change to other boards or to shared display code.

## Anti-goals

- Editing `main/display/*`, `main/application.*` or other upstream files (owner's CLAUDE.md).
- Losing the activation-code or Wi-Fi-setup screens.
- Stutter in audio, wake-word misses, or a noticeable drop in internal SRAM: the minimum is
  ~37 KB today. Animation must stay cheap: a small buffer and modest CPU on the LVGL task.
- Eyes that look frozen: a static image is exactly what the owner dislikes.

## Constraints

- Board-only code in `main/boards/robot-emo/` (the board CMake globs `*.cc`).
- `OledDisplay` (`main/display/oled_display.{h,cc}`) keeps its UI members private, but
  `SetupUI`, `SetEmotion`, `SetChatMessage` and `SetPowerSaveMode` are public virtuals.
  `SetStatus` and `ShowNotification` are virtual in `LvglDisplay`. A board subclass can
  override these, call the base, and add its own LVGL object on top.
- LVGL calls must hold the display lock (`DisplayLockGuard`). Display methods may be
  called from tasks other than the LVGL task.
- The application calls, among others:
  - `SetEmotion("neutral")` when going idle.
  - `SetChatMessage("system", ...)` for activation and alerts, and `SetChatMessage("system", "")`
    to clear.
  - `ShowNotification(...)` for transient messages.
  - `SetStatus(...)` for state text.
- Emotion names (from the firmware's emoji table): neutral, happy, laughing, funny, sad,
  angry, crying, loving, embarrassed, surprised, shocked, thinking, winking, cool, relaxed,
  delicious, kissy, confident, sleepy, silly, confused.
- The board's `config.h` sets `DISPLAY_MIRROR_X`/`DISPLAY_MIRROR_Y`. The panel driver already
  applies them, so drawing code works in normal screen coordinates.

## Approaches considered

1. **Own 1-bit renderer on a 128x64 LVGL canvas, animated by an `lv_timer`** (chosen).
   Full control over shapes (rounded eyes, slanted lids, happy arcs, hearts, tears), smooth
   interpolation, about a 1 KB frame buffer. More drawing code than option 2.
2. LVGL widget objects (rounded rectangles plus black mask objects animated with `lv_anim`).
   Rejected: happy arcs, slanted lids and hearts are awkward, and the result looks stiffer,
   which is the problem being solved.
3. Drawing straight to the panel and bypassing LVGL. Rejected: loses the stock setup and
   activation screens, and fights LVGL for the panel.
4. Pre-rendered bitmap animations. Rejected: large flash cost and canned, less lively motion.

## Design (decided)

- **`EyesDisplay : public OledDisplay`** in `main/boards/robot-emo/`. The board creates it
  instead of `OledDisplay`, with the same constructor arguments.
- `SetupUI()` calls `OledDisplay::SetupUI()`, then creates a full-screen black canvas on
  top of the stock UI. A periodic `lv_timer` (~25 fps) runs on the LVGL task.
- **Eye model.** Per-frame parameters: width/height scale, corner radius, inner-lid slant
  (angry), outer-lid droop (sad), happy-arc amount, look x/y, left-eye closed (wink),
  right-eye shrink (confused/thinking). Each emotion maps to target parameters; the
  current values ease toward the target each frame. On top of that:
  - Random blinks.
  - Random glances when idle.
  - Listening and speaking modifiers, as in Goals.
  - Small extras: heart eyes (loving), a falling tear (crying), "z" (sleepy), "?" (confused),
    dots (thinking), a sunglasses bar (cool).
- **Emotion mapping** (21 names to expressions). Several names share an expression:
  - funny and silly look like laughing.
  - shocked looks like surprised.
  - embarrassed, relaxed and confident are variants of happy or neutral.
  - kissy looks like loving.
  - delicious looks like happy.
  - An unknown name gives neutral. This includes "robot_2", which the app sets at startup
    and which falls back to neutral anyway.
- **Visibility rule** (the eyes win whenever nothing needs reading). The eye canvas is
  hidden, and the stock UI underneath shows through unchanged, when either:
  1. The device state (`Application::GetInstance().GetDeviceState()`, read on the LVGL
     task each frame) is not idle, connecting, listening or speaking. This covers startup,
     Wi-Fi provisioning, activation, upgrading and fatal errors.
  2. A non-empty system message is active: from `SetChatMessage("system", text)` until
     `SetChatMessage("system", "")`. This covers alerts and errors (`Alert()`),
     "please wait", OTA text and the boot user-agent line, even while the state is idle.

  `ShowNotification` alone never hides the eyes. Notifications are brief routine notices
  ("connected", "scanning Wi-Fi"), and cutting to text for each one would make the eyes
  flicker. The long setup-time notifications are already covered by rule 1. When the two
  rules disagree, hiding wins.
- **Power save.** This board never turns the panel off. The base `SetPowerSaveMode(true)`
  only sets the emotion to "sleepy", so the eyes simply show sleepy and the timer keeps
  running.
- **Drawing cost.** The OLED is on I2C at 400 kHz, so a full 1 KB frame takes about 25 ms
  to send. The renderer therefore:
  - draws into its own 1-bit buffer;
  - invalidates the canvas only when the new frame differs from the previous one (the eyes
    are often still between blinks and glances);
  - targets 20 fps, which the builder may lower.

  While the eyes are visible, the stock UI objects underneath stay as they are, fully
  covered by the opaque canvas. Because they don't change, LVGL does not redraw them.
- **Combining modifiers.** Listening and speaking modify any emotion; they never replace it.
  - Listening: size x1.08, look centred, no glancing.
  - Speaking: a vertical bounce added to the emotion's shape (crying while speaking keeps
    the tear and the bounce).
- The emotion glyph and chat label underneath keep receiving updates through the base
  class, so falling back to the stock UI always shows current text.

## Left to the builder

- Exact eye sizes, easing speed and blink and glance timing. Start from the mockup: eye
  base 34 px, easing 0.18 per frame, blink 140 ms every 1.8-5.3 s.
- How the device state is read (`Application::GetInstance().GetDeviceState()`, or tracked
  from `SetStatus`), as long as setup and activation screens stay visible.
- The canvas pixel format (1-bit indexed preferred) and the raster helpers.
- Whether the frame rate is 20 or 25 fps, judged by measured CPU and smoothness.

## Validation

- Build `robot-emo`.
- On hardware (owner):
  - At boot, the stock startup and activation screens are readable.
  - At idle, the eyes blink and glance.
  - Saying "Hi ESP": listening eyes, then speaking bounce, then emotions change with the
    reply. Asking for something sad or funny shows those expressions.
  - Wi-Fi provisioning mode (BOOT at startup) still shows the setup text.
  - The `SystemInfo` minimal SRAM stays close to today's ~37 KB.
  - Audio has no glitches and the wake word still triggers.

# Robot Emo

Custom ESP32-S3-DevKitC-1 board profile for the N16R8 module, based on the
Bread Compact Wi-Fi board implementation.

## Hardware

- INMP441 microphone: WS GPIO4, SCK GPIO5, DIN GPIO6
- MAX98357A speaker: DOUT GPIO7, BCLK GPIO15, LRCK GPIO16
- SSD1306 128x64 OLED: SDA GPIO41, SCL GPIO42, I2C address `0x3C`
- BOOT button: GPIO0
- Optional on-board LED: GPIO48
- L298N motor inputs: IN1-IN4 on GPIO11-GPIO14; leave ENA/ENB unconnected.
  Motor A is IN1/IN2 (GPIO11/12), motor B is IN3/IN4 (GPIO13/14).
  Power the motors separately and connect the motor-supply ground to ESP32 GND.
- MAX30102 heart-rate sensor: second I2C controller (I2C_NUM_1), SDA GPIO8,
  SCL GPIO9, address `0x57`.

The motor outputs are driven LOW during board initialization. Motor movement
and stop, plus heart-rate measurement, are model-visible MCP tools. A missing
MAX30102 does not prevent boot; its tool reports the initialization error.
The heart-rate sampler uses a 5120-byte low-priority task stack; motor
auto-stop uses `esp_timer` without a motor worker task.

## MCP tools

| Tool | Parameters | Behavior |
| --- | --- | --- |
| `self.robot.move` | `direction`: `forward`, `backward`, `left` or `right`; `duration_ms`: 500-5000, default 3000 | Starts the movement and auto-stops after `duration_ms`. Refused while Wi-Fi is disconnected. |
| `self.robot.stop` | none | Sets all four motor inputs LOW immediately. |
| `self.sensor.get_heart_rate` | none | Returns immediately with the latest fresh BPM, or "No fresh reading yet ... Beats detected: N/4" asking the user to keep a finger still on the sensor and ask again in about 5 seconds. If the MAX30102 is missing, retries init (at most once per 2 s) and returns the last initialization error. |

The sensor task samples continuously, so the tool never blocks the main event
loop. Four beat intervals are needed for a BPM value; a reading is stale after
2.5 s without a beat.

### Heart-rate diagnostics

These logs are temporary diagnostics ("DIAGNOSTIC, REMOVE LATER" in
`robot_emo_board.cc`). They need no tool call.

**Data path.** Every 2 s, always on:

```
I (...) RobotEmoBoard: HRALIVE polls=200 data_polls=50 samples=50 recoveries=0 recoveries_total=0 ir=1834 red=1512 ir_raw=00072A red_raw=0005E8 wr=7 rd=7 ovf=0x00 r08=0x4F r09=0x03 r0A=0x27 r0C=0x3C r0D=0x3C id=0x15 i2c_err=0
```

Expected when the sensor works: about 50 `samples` per 2 s (25/s), `ovf=0x00`
or small, `ir_raw`/`red_raw` non-zero, the registers as configured, `id=0x15`
and `i2c_err=0`. `ir` is low (a few thousand) with no finger and rises well
above 50000 with a finger resting on the sensor.

- `samples=0` with `ovf=0x1F` and `wr` equal to `rd` means the FIFO is full
  and not being read. The firmware now detects this (non-zero overflow
  counter), drains the stale samples, resets the FIFO and logs
  `MAX30102 FIFO overflow (ovf=..): drained .. stale samples, cleared FIFO`;
  `recoveries` counts this. One recovery right after boot is harmless; repeated
  recoveries mean the sensor task is being starved.
- `polls`/`data_polls`: FIFO checks in the interval, and how many found data.
  Each check reads every waiting sample.

The first five raw samples after start-up are logged once to check the byte
order (3 bytes RED, then 3 bytes IR):

```
I (...) RobotEmoBoard: HRFIRST 1: 00 05 E8 00 07 2A (red=1512 ir=1834)
```

**Signal summary.** While the raw IR value is above 10000, the log prints one
line per second:

```
I (...) RobotEmoBoard: HRDIAG ir_min=118000 ir_max=119500 baseline=118700 ac_min=-600 ac_max=700 peak_thr=593 samples=25 beats=2/4 finger=1
```

- `ir_min`/`ir_max`: raw 18-bit IR range over that second. Near 262143 means
  the ADC is saturated: lower `kLedCurrent`.
- `baseline`: slow moving average of IR (0 when no finger is detected).
- `ac_min`/`ac_max`: IR minus baseline over that second (0 when no finger).
  The pulse is the swing between them.
- `peak_thr`: how far above the baseline the signal must rise to count a beat
  (baseline/200, at least 200). If `ac_max` stays below it, no beats are
  counted.
- `samples`: IR samples in that second (about 25 at the current settings).
- `beats=N/4`: beat intervals collected; a BPM needs 4.
- `finger`: 1 when IR is above `kFingerThreshold` (50000).

**Raw capture.** With `kHeartRateRawCapture = true`, the first 20 s after a
finger is placed are also logged as one line per second:

```
I (...) RobotEmoBoard: HRRAW,1000,118200,118350,...
```

The first number is the time of that line's first sample in ms since the
capture started; the rest are the raw IR samples of that second. Copy the
lines into a spreadsheet to plot the waveform. The capture re-arms after IR
stays at or below 10000 for 3 s (finger removed).

**Sensor not found.** Init is retried: three attempts at boot about 50 ms
apart, then at most one attempt per 2 s when `self.sensor.get_heart_rate` is
called. The tool's error message includes the last init error. Each failed
attempt logs:

```
W (...) RobotEmoBoard: MAX30102 init attempt 1/3 failed: MAX30102 not found at I2C address 0x57
W (...) RobotEmoBoard: MAX30102 diag: SDA(GPIO8)=1 SCL(GPIO9)=1 with internal pull-ups
W (...) RobotEmoBoard: MAX30102 diag: I2C scan 0x08-0x77 ACK: none
```

- A level of 0 means that line is held LOW (short, or a powered-down module):
  the scan is skipped.
- `ACK: none` with both lines at 1 means nothing answers: check power, GND and
  the SDA/SCL wires.
- A different address answering means a different chip or a wrong wire.
- `device at 0x57 has part ID 0x.., expected 0x15` means the chip at 0x57 is
  not a MAX30102.

The sensor I2C bus runs at 100 kHz. The LED current is `kLedCurrent` (0x3C,
about 12 mA) for both LEDs.

## Build and flash on Windows

From an ESP-IDF PowerShell with ESP-IDF v6.0.3 exported:

```powershell
Set-Location C:\dev\xiaozhi-esp32
idf.py --version
python scripts\build.py robot-emo --name robot-emo --language vi-VN
idf.py -p COMx flash monitor
```

Replace `COMx` with the device's serial port. The ESP32-S3 defaults already
select 16 MB flash and octal PSRAM; this board only overrides the OLED
resolution. Pass `--language vi-VN` when building to select Vietnamese.

## Motor direction flags

The motors are mounted mirrored, so firmware inverts one of them instead of
rewiring. Two constants near the top of `robot_emo_board.cc` control this:

- `kInvertMotorA = false`: motor A (IN1/IN2, GPIO11/12)
- `kInvertMotorB = true`: motor B (IN3/IN4, GPIO13/14)

`true` swaps that motor's two input levels for every direction (forward,
backward, left, right). The direction table in `MotorDriver::Move` stays
logical; the inversion is applied after it. If one wheel turns the wrong way on
"forward", flip that motor's flag; if both do, flip both. Rebuild and repeat
the safe motor test below after any change.

## Safe motor test

1. Raise the robot so all wheels are off the ground. Disconnect the L298N motor
   supply first; keep ESP32 GND and L298N GND common. Flash and open the serial
   monitor before reconnecting motor power.
2. With motor power still disconnected, issue the move and stop phrases below.
   Confirm `self.robot.move`/`self.robot.stop` argument/result log entries and
   the auto-stop log appear. Disconnect Wi-Fi during a move and confirm the
   safety-stop log appears.
3. Only after those checks, reconnect motor power while keeping the wheels off
   the ground. Test each direction with the minimum 500 ms duration, verify
   forward/back/left/right match the wiring table, then verify stop.

Vietnamese voice tests:

- “Robot, tiến lên trong 1 giây.”
- “Robot, lùi lại trong 1 giây.”
- “Robot, quay trái trong 1 giây.”
- “Robot, quay phải trong 1 giây.”
- “Robot, dừng lại.”
- “Đo nhịp tim của tôi.”

Heart-rate results are reference values only, not medical measurements.

## Controls

During startup, click BOOT to enter Wi-Fi provisioning. After startup, click
BOOT to toggle chat/listening, matching the Bread Compact Wi-Fi behavior.

## Wake word

The default wake word is the stock ESP-SR WakeNet "Hi ESP" (`wn9_hiesp`).
Say it as English: "hi, E-S-P". A plain build selects it:

```powershell
python scripts\build.py robot-emo --name robot-emo --language vi-VN
```

BOOT click still toggles chat as a fallback. The boot log shows
`Initialized FD AFE, detector: WakeNet, ...`, and each detection logs
`Application: Wake word detected: ...`.

### Optional: custom MultiNet wake word (experimental)

`config.json` also keeps an experimental MultiNet custom wake word
("alo tí ơi", model `mn7_cn`). It is used only when you opt in:

```powershell
python scripts\build.py robot-emo --name robot-emo --language vi-VN --wake-word custom
```

Without `--wake-word custom`, `scripts/build.py` turns the board's MultiNet
model off, so WakeNet runs and no MultiNet model is packed into the assets.

In testing on this robot, none of the custom Vietnamese spellings triggered,
even at threshold 10. ESP-SR has no Vietnamese model, so the phrase is only
approximated with toneless pinyin.

Settings in `config.json` (they take effect only with the opt-in):

- `CONFIG_SR_MN_CN_MULTINET7_QUANT=y` selects the `mn7_cn` model.
- `CONFIG_CUSTOM_WAKE_WORD` holds the spellings, separated by `|`. Each must be
  toneless pinyin syllables separated by spaces (no Vietnamese letters). Each
  spelling becomes its own wake command, numbered from 1 in order. Currently
  1 `a lou ti e yi`, 2 `a luo ti ei`, 3 `a lou ti ei`, 4 `dai ga`, 5 `dai ka`,
  6 `da ga`, 7 `da ge`, 8 `dai ge`.
- `CONFIG_CUSTOM_WAKE_WORD_DISPLAY` is the text sent to the server on wake.
- `CONFIG_CUSTOM_WAKE_WORD_THRESHOLD` is 1-99 (divided by 100). Lower is more
  sensitive. It is currently 10 for testing; use about 20 or higher for normal
  use.

Serial log lines for a custom build:

- `CustomWakeWord: Command: <spelling>, Text: alo tí ơi, Action: wake` at boot,
  one per spelling, and `Initialized FD AFE, detector: MultiNet, ...`.
- `CustomWakeWord: Custom wake word detected: command_id=N, string=..., prob=...`
  on every detection (N is the spelling's number). Only detections at or above
  the threshold are logged; near misses are not visible through ESP-SR's API.
- `RobotEmoBoard: Internal SRAM (after audio engine start|20 s after
  conversation end): min free=... free now=...` (a temporary diagnostic) to
  compare memory use with the WakeNet build.

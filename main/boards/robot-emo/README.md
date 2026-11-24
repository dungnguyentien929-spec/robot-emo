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
  Power the motors separately and connect the motor-supply ground to ESP32 GND.
- MAX30102 heart-rate sensor: second I2C controller (I2C_NUM_1), SDA GPIO8,
  SCL GPIO9, address `0x57`.

The motor outputs are driven LOW during board initialization. Motor movement
and stop, plus heart-rate measurement, are model-visible MCP tools. A missing
MAX30102 does not prevent boot; its tool reports the initialization error.
The heart-rate sampler uses a 4096-byte low-priority task stack; motor
auto-stop uses `esp_timer` without a motor worker task.

## MCP tools

| Tool | Parameters | Behavior |
| --- | --- | --- |
| `self.robot.move` | `direction`: `forward`, `backward`, `left` or `right`; `duration_ms`: 500-5000, default 3000 | Starts the movement and auto-stops after `duration_ms`. Refused while Wi-Fi is disconnected. |
| `self.robot.stop` | none | Sets all four motor inputs LOW immediately. |
| `self.sensor.get_heart_rate` | none | Returns immediately with the latest fresh BPM, or "No fresh reading yet ... Beats detected: N/4" asking the user to keep a finger still on the sensor and ask again in about 5 seconds. If the MAX30102 is missing, returns its initialization error. |

The sensor task samples continuously, so the tool never blocks the main event
loop. Four beat intervals are needed for a BPM value; a reading is stale after
2.5 s without a beat.

### Heart-rate diagnostics

After each `self.sensor.get_heart_rate` call, the serial log prints one line
per second for 15 seconds:

```
I (...) RobotEmoBoard: MAX30102 diag sample: IR=123456 baseline=123000 finger_threshold=50000
```

- `IR` is the latest raw 18-bit IR sample; `baseline` is its slow moving
  average (0 when no finger is detected).
- With no finger, `IR` should stay below `finger_threshold`
  (`kFingerThreshold`). With a finger resting still, it should be well above
  it and steady. If it is not, adjust `kFingerThreshold` or the LED current
  registers (0x0C/0x0D, currently `0x24`) in `robot_emo_board.cc`.
- `IR` near 262143 means the ADC is saturated: lower the LED current.

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

## Wake word: "alo tí ơi"

This board replaces the stock WakeNet "Hi ESP" with a MultiNet custom wake
word (`mn7_cn`). BOOT click still toggles chat as a fallback. ESP-SR has no
Vietnamese model, so the phrase is approximated with toneless pinyin. Three
spellings are registered as separate wake commands, so the log shows which one
matched.

It is configured in `config.json`:

- `"wake_word": "custom"` makes `scripts/build.py` select
  `CONFIG_USE_CUSTOM_WAKE_WORD` instead of a WakeNet model. `--wake-word`
  on the command line still overrides it.
- `CONFIG_SR_MN_CN_MULTINET7_QUANT=y` packs `mn7_cn` into the assets.
- `CONFIG_CUSTOM_WAKE_WORD` holds the spellings, separated by `|`. Each must be
  valid toneless pinyin syllables separated by spaces (no Vietnamese letters).
  Keep the best one or two once you have test results; fewer commands means
  fewer false triggers.
- `CONFIG_CUSTOM_WAKE_WORD_DISPLAY` is the text sent to the server on wake.
- `CONFIG_CUSTOM_WAKE_WORD_THRESHOLD` is 1-99 (divided by 100). Lower is more
  sensitive. Start at 20; raise it if there are false triggers, lower it if
  real attempts are missed.

Rebuild after any change: the build regenerates `index.json` and
`generated_assets.bin`, and `idf.py flash` writes both the app and the assets
partition.

Serial log lines to watch:

- `CustomWakeWord: Command: <spelling>, Text: alo tí ơi, Action: wake` at boot,
  one per spelling.
- `CustomWakeWord: Custom wake word detected: command_id=N, string=..., prob=...`
  on every detection. N is the spelling's position (1-3). Detections while no
  one said the phrase are false triggers.
- `Initialized FD AFE, detector: MultiNet, ...` confirms MultiNet replaced
  WakeNet; the `After AFE create: free=... min=...` line right after it also
  shows internal SRAM.
- `RobotEmoBoard: Internal SRAM (after audio engine start|20 s after
  conversation end): min free=... free now=...` to compare with the ~48 KB
  minimum free internal SRAM seen with WakeNet.

Suggested test: 20 attempts per spelling at about 1 m and 3 m, then 10 minutes
of TV or conversation nearby with no wake attempts. Count detections per
`command_id` and false triggers, then adjust the spellings and threshold.

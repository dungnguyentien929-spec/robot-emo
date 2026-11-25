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
| `self.sensor.get_heart_rate` | none | Returns immediately with the current BPM. If the current 4-beat window is not steady, it returns the last steady reading from the past 10 s while the finger stays on the sensor. Otherwise it returns "No fresh reading yet ... Beats detected: N/4", asking the user to keep a finger still on the sensor and ask again in about 5 seconds. If the MAX30102 is missing, it retries init (at most once per 2 s) and returns the last initialization error. |
| `self.weather.get` | none | Returns immediately with the cached current weather and today's and tomorrow's forecast for Thủ Đức City. If no data has been fetched yet, it returns an error saying why. Data older than 2 h is marked STALE. |

The sensor task samples continuously, so the tool never blocks the main event
loop.

### Heart-rate measurement

**Signal.** A resting finger reads about 62000 IR. The pulse is only about
100-200 counts (~0.3%). Each heartbeat shows as a sharp drop within ~80 ms,
followed by a slow rise. With no finger, IR stays far below the finger threshold,
8000 (`kFingerThreshold`).

**Beat detection.** A beat is counted when the 80 ms slope, after removing slow
drift, falls below a threshold. The threshold is half the strongest recent
drop, and at least IR/2000 or 20 counts. A BPM needs four beat intervals that
agree within 25%. Motion gives irregular intervals, so then there is no current
reading rather than a wrong number. One missed or extra beat breaks the window
for a few beats, so the tool falls back to the last steady reading from the
past 10 s while the finger stays on. A reading is stale after 2.5 s without a
beat. Removing the finger clears everything.

**Sensor quirks** (found on this module, kept in the driver):

- On this sensor, writing 0 to the FIFO pointer registers does **not** empty a
  full FIFO. It stays full (`FIFO_WR_PTR == FIFO_RD_PTR`, `OVF_COUNTER`
  climbing), and every new sample is dropped until entries are read from
  FIFO_DATA. The firmware never clears the FIFO by register writes. When
  `OVF_COUNTER` is non-zero, it reads and discards all 32 entries. This happens
  once at boot, because the FIFO starts full of garbage.
- Real samples have bits 23:18 set (for example IR `3B FF FF`), so the driver
  keeps only bits 17:0.
- At the 4096 nA ADC range, a firmly pressed finger saturated IR at 262143. The
  range is now 16384 nA (`kSpo2Config` = 0x67). If `ir` ever sits at 262143
  again, lower `kLedCurrent`.

**Logs.** These appear in normal operation:

```
I (...) RobotEmoBoard: Heart rate: finger detected (IR 61913)
I (...) RobotEmoBoard: MCP self.sensor.get_heart_rate args={} result=BPM: 64. This is a reference value, not medical.
I (...) RobotEmoBoard: Heart rate: finger removed (IR 812)
```

The following warning prints at most once per 10 s, and only when something
happened:

```
W (...) RobotEmoBoard: MAX30102: 1 full-FIFO drains, 0 I2C errors since the last warning
```

One drain right after boot is expected. Repeated drains mean the sensor task
is being starved.

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
about 12 mA) for both LEDs; the ADC range is 16384 nA (`kSpo2Config`, 0x67).

### Weather

`self.weather.get` answers for one fixed place: Thủ Đức City (latitude 10.85,
longitude 106.77, time zone Asia/Ho_Chi_Minh). The data comes from
[Open-Meteo](https://open-meteo.com/) over HTTPS, with no account or API key.

- A low-priority `weather` task, with an 8 KB stack in PSRAM, waits for Wi-Fi.
  It then fetches every 15 minutes, or 1 minute after a failure. A failed
  fetch keeps the previous data.
- The tool only reads that cache, so it never waits on the network. The answer
  includes the current temperature, feels-like temperature, humidity, sky
  condition, wind and precipitation. It also gives today's and tomorrow's
  high/low, condition and rain chance, each labelled with its date, and how long
  ago the data was fetched. The dates let the model notice when a day has
  rolled over since the last fetch.
  The model says it in Vietnamese.
- To change the place, edit `kWeatherPlace`, then the latitude and longitude
  in `kWeatherUrl` near the top of `robot_emo_board.cc`, and rebuild. Outside
  Vietnam, also change the URL's `timezone`, which sets where "today" starts.
- A fetch cannot hang: esp-tls gives the socket a 10 s send/receive timeout
  even though the HTTP client sets none for the TLS handshake. A stalled
  connection fails and is retried a minute later.
- Parsing and the answer text live in `weather_report.{h,cc}`. They have host
  tests in `scripts/tests/test_robot_emo_weather.py`, which needs a host C++23
  compiler and is skipped otherwise.

Log lines:

```
I (...) RobotEmoBoard: Weather: 31.4 C, partly cloudy (fetched in 850 ms)
I (...) RobotEmoBoard: Weather task stack: 3100 of 8192 bytes never used
W (...) RobotEmoBoard: Weather fetch failed: connect failed: ...; retrying in 1 min
I (...) RobotEmoBoard: MCP self.weather.get args={} result=Weather for Thu Duc City, ...
```

The stack line appears once, after the first successful fetch. Use it to tune
`kStackBytes`.

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
- “Thời tiết hôm nay thế nào?”
- “Mai có mưa không?”

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

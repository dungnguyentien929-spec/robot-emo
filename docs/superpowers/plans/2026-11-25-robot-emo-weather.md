# robot-emo Weather Tool Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The robot answers questions about the current weather and today's and tomorrow's forecast for Thủ Đức, instantly, from a cache that a background task refreshes from Open-Meteo.

**Architecture:** A pure, host-testable module parses the Open-Meteo JSON into a struct and formats the tool's answer text. A `WeatherService` class in the board file owns a low-priority task (stack in PSRAM) that fetches over HTTPS every 15 min and stores the result under a mutex. The MCP tool `self.weather.get` only reads that cache.

**Tech Stack:** ESP-IDF v6.0.3, C++23 (`std::expected`), cJSON, FreeRTOS, the board `NetworkInterface::CreateHttp`, Python `unittest` and a host `c++` for the parser test.

**Spec:** `docs/superpowers/specs/2026-11-25-robot-emo-weather-design.md`

## Global Constraints

- Board-only: all code in `main/boards/robot-emo/`. No change to `main/mcp_server.*`, `main/application.*`, display, or protocols.
- The tool never touches the network; it reads the cache and returns immediately.
- Place: Thủ Đức City, latitude 10.85, longitude 106.77, time zone `Asia/Ho_Chi_Minh`.
- URL: `https://api.open-meteo.com/v1/forecast?latitude=10.85&longitude=106.77&current=temperature_2m,relative_humidity_2m,apparent_temperature,precipitation,weather_code,wind_speed_10m&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max&timezone=Asia%2FHo_Chi_Minh&forecast_days=2`
- Refresh every 15 min after success, 1 min after failure. Data older than 2 h is reported as stale, with its age.
- No data: the tool returns an error naming the last failure; it never invents values.
- A weather failure must not affect boot, audio, motors, the heart-rate sensor or the conversation.
- Free internal SRAM minimum stays near today's ~40 KB: task stack in PSRAM, TLS already allocates from PSRAM.
- Keep `main/boards/robot-emo/README.md` in sync.

## Review Focus

- Open-Meteo returns HTTP 200 but with a missing or `null` field (e.g. `precipitation_probability_max` is `null`): the parse must not crash. A missing rain probability is reported as unknown; a missing required current value is a parse error. Tests: `missing_probability_is_unknown`, `missing_current_is_error`.
- Malformed or truncated body (Wi-Fi drop mid-read): `ParseOpenMeteo` returns an error, the old cache is kept, retry in 1 min. Test: `truncated_json_is_error`.
- Wi-Fi not connected at boot or lost later: the task waits without fetching or logging every second; the tool says no data / stale with age. Checked on hardware (Task 2).
- An unknown WMO code (a code not in the table): the text says `unknown (code N)` and does not crash. Test: `unknown_code_text`.
- The tool asked before the first fetch completes: a clear "not fetched yet" error rather than zeros. Checked on hardware (Task 2) and by the `has_data` guard in `GetText`.

---

### Task 1: Parser and formatter (pure, host-tested)

**Files:**
- Create: `main/boards/robot-emo/weather_report.h`, `main/boards/robot-emo/weather_report.cc` (the board CMake globs `boards/${BOARD_DIR}/*.cc`, so no CMake change)
- Test: `scripts/tests/test_robot_emo_weather.py`

**Interfaces:**
- Produces (namespace `robot_emo`):
  - `struct DailyForecast { int weather_code; float temp_max_c; float temp_min_c; int rain_probability_pct; /* -1 = unknown */ };`
  - `struct WeatherReport { float temperature_c; float feels_like_c; int humidity_pct; float precipitation_mm; int weather_code; float wind_kmh; DailyForecast today; DailyForecast tomorrow; };`
  - `std::expected<WeatherReport, std::string> ParseOpenMeteo(const std::string& body);` uses cJSON and frees everything it parses.
  - `const char* WmoDescription(int code);` returns the standard Open-Meteo text ("clear sky", "mainly clear", "partly cloudy", "overcast", "fog", "light drizzle", ..., "thunderstorm with heavy hail"), or `nullptr` for an unknown code.
  - `std::string FormatWeather(const WeatherReport& report, const char* place, int64_t age_s);` returns English text for the model.

- [ ] **Step 1: Write the failing test.** Follow `scripts/tests/test_mcp_server.py`: compile with `c++ -std=c++23`, with `weather_report.cc` and `managed_components/espressif__cjson/cJSON/cJSON.c`, plus a small `main()` that `assert`s. Skip with `unittest.SkipTest` if no `c++` or no C++23 `<expected>`. The fixture body is a real-shaped Open-Meteo response: current temperature 31.4, apparent 36.2, humidity 70, precipitation 0.0, weather_code 2, wind 9.7; daily weather_code [80, 95], max [33.1, 32.0], min [25.2, 25.0], probability [60, null]. Cases:
  - `parses_current_and_two_days`: the values above, `today.rain_probability_pct == 60`.
  - `missing_probability_is_unknown`: `tomorrow.rain_probability_pct == -1`.
  - `missing_current_is_error`: a body without `current.temperature_2m` returns an error.
  - `truncated_json_is_error`: the fixture cut in half returns an error.
  - `unknown_code_text`: `WmoDescription(42) == nullptr`, and `FormatWeather` with `weather_code = 42` contains `"unknown (code 42)"`.
  - `format_contents`: `FormatWeather(r, "Thu Duc City", 300)` contains `"Thu Duc City"`, `"31.4"`, `"partly cloudy"`, `"Tomorrow"`, `"thunderstorm"`, `"rain chance unknown"` and `"5 minutes ago"`. With `age_s = 3 * 3600` it contains `"stale"`.

- [ ] **Step 2: Run the test and confirm it fails.** Run `set PYTHONUTF8=1` then `python -m unittest scripts.tests.test_robot_emo_weather -v`. Expected: a compile failure because `weather_report.h` is missing, or SKIP where there is no host compiler. On a skip, note it and rely on Task 2's firmware build for compile coverage.

- [ ] **Step 3: Implement `ParseOpenMeteo`, `WmoDescription` and `FormatWeather`.** Required numbers come from `current.*` and from `daily.*[0]` and `daily.*[1]`. A missing or non-number required field returns `std::unexpected("missing <field>")`. A `null` or missing probability gives -1. Age wording: under 60 s "just now", otherwise "N minutes ago". At 2 h or more, prefix "STALE (fetched N minutes ago)".

- [ ] **Step 4: Run the test and confirm it passes** (same command). Expected: OK, or SKIP for the stated reason.

- [ ] **Step 5: Commit** `weather_report.{h,cc}` and the test: `robot-emo: weather report parser and formatter`.

### Task 2: Background fetcher, MCP tool, README

**Files:**
- Modify: `main/boards/robot-emo/robot_emo_board.cc`: add a `WeatherService` class after `HeartRateSensor`, a member in `RobotEmoBoard`, and the tool registration next to `self.sensor.get_heart_rate`.
- Modify: `main/boards/robot-emo/README.md`: the tool table row and a short "Weather" section (source, place, refresh, how to change the place, log lines).

**Interfaces:**
- Consumes: `robot_emo::ParseOpenMeteo`, `robot_emo::FormatWeather`, `robot_emo::WeatherReport` from Task 1.
- Produces: `class WeatherService { public: void Start(); ToolResult GetText(); };`. `Start()` creates the task once; `GetText()` is non-blocking.

- [ ] **Step 1: Add the constants** near the other board constants: `kWeatherLatitude = 10.85`, `kWeatherLongitude = 106.77`, `kWeatherPlace = "Thu Duc City, Ho Chi Minh City, Vietnam"`, and the URL from Global Constraints built from them.

- [ ] **Step 2: Implement `WeatherService`.**
  - Task: `xTaskCreatePinnedToCoreWithCaps(..., "weather", 8192, this, 1, &task_, tskNO_AFFINITY, MALLOC_CAP_SPIRAM)`. If that fails, log an error and keep running without weather.
  - Loop: while `!WifiManager::GetInstance().IsConnected()`, `vTaskDelay(5 s)`. Then fetch with `Board::GetInstance().GetNetwork()->CreateHttp(3)`, `Open("GET", url)`, check `GetStatusCode() == 200`, `ReadAll()`, `Close()`. Pattern: `self.screen.preview_image` in `main/mcp_server.cc`.
  - Cap the body at 4 KB: a larger body is an error.
  - On success, store the report and `esp_timer_get_time()` under the mutex, log `Weather: 31.4 C, partly cloudy (fetched in N ms)`, and sleep 15 min. On failure, store the error string only (the old report is kept), log a warning, and sleep 1 min.
  - `GetText()`: under the mutex, with no report, returns `std::unexpected("Weather not fetched yet" + (last error ? ": " + error : ", waiting for Wi-Fi"))`. Otherwise it returns `FormatWeather(report, kWeatherPlace, age_s)`.
  - Log the task's stack high-water mark once, after the first successful fetch, so the 8 KB can be tuned.

- [ ] **Step 3: Register the tool** `self.weather.get`, no parameters. Description: "Current weather plus today's and tomorrow's forecast for where the robot is (Thu Duc City). Use for any question about the weather, temperature, rain or tomorrow's weather here. Returns immediately from a cache refreshed every 15 minutes." Log args/result the way the heart-rate tool does. Call `weather_.Start()` from the board constructor after the tools are registered.

- [ ] **Step 4: Update the README** (tool row and Weather section).

- [ ] **Step 5: Build.** Activate with `. C:\Espressif\tools\Microsoft.v6.0.3.PowerShell_profile.ps1`, then run `python scripts\build.py robot-emo --name robot-emo --language vi-VN`. Expected: `Project build complete`, and no warnings from `robot_emo_board.cc` or `weather_report.cc`. Run clang-format only on the new and changed ranges.

- [ ] **Step 6: Hardware check (user).** Confirm all of these:
  - `Weather: ...` appears within seconds of `Got IP`.
  - "thời tiết hôm nay thế nào?" and "mai có mưa không?" get plausible answers.
  - Booting with the Wi-Fi off makes the tool answer "not fetched yet", and nothing else breaks.
  - `SystemInfo` minimal SRAM stays near 40 KB.
  - The stack high-water log shows headroom.

- [ ] **Step 7: Commit:** `robot-emo: weather tool for Thu Duc (Open-Meteo, cached)`.

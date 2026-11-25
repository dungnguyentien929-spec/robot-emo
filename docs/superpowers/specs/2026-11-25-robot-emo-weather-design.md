# robot-emo: weather tool (Thủ Đức)

Status: approved 2026-11-25. Size: small change, board-only.

## Intent

The robot's owner asks the robot about the weather ("thời tiết hôm nay thế
nào?", "mai có mưa không?") and gets a spoken answer. Today the robot cannot
answer: the firmware has no weather tool, and the server-side AI only does
what the device's MCP tools offer.

## Goals

- Current weather and today's and tomorrow's forecast for one fixed place:
  Thủ Đức City, about 10.85° N, 106.77° E, time zone Asia/Ho_Chi_Minh.
- The answer is instant: the tool never waits on the network.
- When no data is available, the tool says so and why (no network yet, fetch
  error). It never invents numbers.

## Non-goals

- Other cities. Fetching on demand would block the main event loop for
  1-3 s (MCP tool callbacks run there via `Application::Schedule`). Doing it
  without blocking needs an asynchronous reply path in the shared MCP server,
  which affects every board. Separate project if wanted.
- Showing weather on the OLED. Not requested; the screen is owned by the
  shared display code, and a weather screen needs its own UX decisions.
- Forecasts beyond tomorrow.

## Anti-goals

- Any change to core modules (`main/mcp_server.*`, `main/application.*`,
  display, protocols).
- A weather failure affecting anything else: boot, audio, motors, the
  heart-rate sensor, or the conversation.
- Noticeable internal-SRAM cost. Free internal SRAM bottoms out near 40 KB
  with the audio engine running.

## Constraints

- Board-specific code stays in `main/boards/robot-emo/`.
- Do not block the main event loop.
- mbedTLS allocates from PSRAM (`CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y`), the
  certificate bundle is enabled, and task stacks may live in PSRAM
  (`CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y`).
- HTTP goes through `Board::GetInstance().GetNetwork()->CreateHttp(...)`, as
  `main/mcp_server.cc` and `main/ota.cc` do.
- Keep the board README in sync (CLAUDE.md).

## Approaches considered

1. **Background fetch into a cache; tool reads the cache.** Chosen. Instant,
   non-blocking, board-only. Works because the place is fixed.
2. Fetch when the tool is called. Rejected: blocks the main loop 1-3 s.
3. Enable a weather feature on the server (xiaozhi.me agent settings).
   Rejected: outside our control, and its availability is unknown.

## Design

- **Source:** Open-Meteo forecast API over HTTPS, no key:
  `https://api.open-meteo.com/v1/forecast?latitude=10.85&longitude=106.77`
  `&current=temperature_2m,relative_humidity_2m,apparent_temperature,precipitation,weather_code,wind_speed_10m`
  `&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max`
  `&timezone=Asia%2FHo_Chi_Minh&forecast_days=2`
  Daily index 0 is today, index 1 is tomorrow.
- **Location:** constants near the top of `robot_emo_board.cc` (latitude,
  longitude, place name). Changing the place means editing them and rebuilding.
- **Fetcher:** one low-priority task with its stack in PSRAM. It waits for
  the network, fetches, then sleeps 15 minutes; after a failure it retries in
  1 minute. It parses with cJSON, freeing everything it creates, and stores a
  small struct under a mutex: values, fetch time, and the last error.
- **Tool:** `self.weather.get` (no parameters) reads the cache and returns
  English text for the model to say in Vietnamese: place, current conditions
  (temperature, feels-like, humidity, condition from the WMO weather code,
  wind, precipitation), today's and tomorrow's high/low, condition and rain
  probability, and how many minutes ago the data was fetched. If there is no
  data, it returns an error naming the last failure. Data older than about
  2 hours is reported as stale, with its age.
- **Logs:** one line per fetch (success with temperature, or the error).

## Left to the builder

- The exact task stack size (measure the high-water mark on hardware), the
  HTTP timeout, and how the task detects that the network is up.
- The WMO code-to-text table (standard Open-Meteo descriptions).
- Tool description wording, so the model calls it for any weather question
  about "here", "today" or "tomorrow".

## Validation

- Build `robot-emo`.
- On hardware: the fetch log appears after Wi-Fi connects. Asking
  "thời tiết hôm nay thế nào?" and "mai có mưa không?" returns plausible
  values. With Wi-Fi off at boot, the tool reports no data instead of failing.
  Internal-SRAM minimum stays close to today's (~40 KB).

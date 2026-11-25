#pragma once

// Open-Meteo weather for robot-emo: parsing and the tool's answer text.
// Pure C++ and cJSON only, so it is host-testable (scripts/tests/test_robot_emo_weather.py).

#include <cstdint>
#include <expected>
#include <string>

namespace robot_emo {

struct DailyForecast {
    std::string date;  // YYYY-MM-DD, local to the place; the model reconciles it with today
    int weather_code = 0;
    float temp_max_c = 0;
    float temp_min_c = 0;
    int rain_probability_pct = -1;  // -1 = unknown
};

struct WeatherReport {
    float temperature_c = 0;
    float feels_like_c = 0;
    int humidity_pct = 0;
    float precipitation_mm = 0;
    int weather_code = 0;
    float wind_kmh = 0;
    DailyForecast today;
    DailyForecast tomorrow;
};

// Parses an Open-Meteo /v1/forecast body requested with the current and daily fields in
// kWeatherUrl (robot_emo_board.cc) and forecast_days=2. A missing, non-finite or out-of-range
// required value is an error (network input).
std::expected<WeatherReport, std::string> ParseOpenMeteo(const std::string& body);

// Open-Meteo's text for a WMO weather code, or nullptr for an unknown code.
const char* WmoDescription(int code);

// English text for the model. age_s: seconds since the data was fetched.
std::string FormatWeather(const WeatherReport& report, const char* place, int64_t age_s);

}  // namespace robot_emo

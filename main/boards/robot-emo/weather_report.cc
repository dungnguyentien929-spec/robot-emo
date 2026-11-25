#include "weather_report.h"

#include <cJSON.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>

namespace robot_emo {
namespace {

using JsonPtr = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;

constexpr int64_t kStaleAfterS = 2 * 3600;
constexpr double kMaxMagnitude = 1e6;

// Network input: only finite numbers of plausible magnitude are accepted, so the later casts
// to int are always defined.
bool ToNumber(const cJSON* item, double* value) {
    if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble) ||
        std::fabs(item->valuedouble) > kMaxMagnitude) {
        return false;
    }
    *value = item->valuedouble;
    return true;
}

bool GetNumber(const cJSON* object, const char* name, double* value) {
    return ToNumber(cJSON_GetObjectItemCaseSensitive(object, name), value);
}

// Element `index` of the numeric array `name` in `object`; false if missing or invalid.
bool GetArrayNumber(const cJSON* object, const char* name, int index, double* value) {
    const cJSON* array = cJSON_GetObjectItemCaseSensitive(object, name);
    return cJSON_IsArray(array) && ToNumber(cJSON_GetArrayItem(array, index), value);
}

std::expected<DailyForecast, std::string> ParseDay(const cJSON* daily, int index) {
    DailyForecast day;
    const cJSON* dates = cJSON_GetObjectItemCaseSensitive(daily, "time");
    const cJSON* date = cJSON_IsArray(dates) ? cJSON_GetArrayItem(dates, index) : nullptr;
    if (!cJSON_IsString(date) || strlen(date->valuestring) != 10) {
        return std::unexpected("missing daily.time[" + std::to_string(index) + "]");
    }
    day.date = date->valuestring;
    double code = 0;
    double max = 0;
    double min = 0;
    if (!GetArrayNumber(daily, "weather_code", index, &code)) {
        return std::unexpected("missing or invalid daily.weather_code[" + std::to_string(index) +
                               "]");
    }
    if (!GetArrayNumber(daily, "temperature_2m_max", index, &max)) {
        return std::unexpected("missing or invalid daily.temperature_2m_max[" +
                               std::to_string(index) + "]");
    }
    if (!GetArrayNumber(daily, "temperature_2m_min", index, &min)) {
        return std::unexpected("missing or invalid daily.temperature_2m_min[" +
                               std::to_string(index) + "]");
    }
    day.weather_code = static_cast<int>(code);
    day.temp_max_c = static_cast<float>(max);
    day.temp_min_c = static_cast<float>(min);
    double probability = 0;
    if (GetArrayNumber(daily, "precipitation_probability_max", index, &probability)) {
        day.rain_probability_pct = static_cast<int>(probability);
    }
    return day;
}

std::string Condition(int code) {
    const char* text = WmoDescription(code);
    return text != nullptr ? std::string(text) : "unknown (code " + std::to_string(code) + ")";
}

std::string Number(float value) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%.1f", value);
    return buffer;
}

std::string DayText(const char* label, const DailyForecast& day) {
    std::string text = std::string(label) + " (" + day.date + "): " + Condition(day.weather_code) +
                       ", high " + Number(day.temp_max_c) + " C, low " + Number(day.temp_min_c) +
                       " C, ";
    text += day.rain_probability_pct >= 0
                ? "rain chance " + std::to_string(day.rain_probability_pct) + "%"
                : std::string("rain chance unknown");
    return text + ".";
}

}  // namespace

std::expected<WeatherReport, std::string> ParseOpenMeteo(const std::string& body) {
    JsonPtr root(cJSON_ParseWithLength(body.data(), body.size()), cJSON_Delete);
    if (root == nullptr) {
        return std::unexpected(std::string("invalid JSON"));
    }
    const cJSON* current = cJSON_GetObjectItemCaseSensitive(root.get(), "current");
    const cJSON* daily = cJSON_GetObjectItemCaseSensitive(root.get(), "daily");
    if (!cJSON_IsObject(current) || !cJSON_IsObject(daily)) {
        return std::unexpected(std::string("missing current or daily"));
    }

    static constexpr const char* kCurrentFields[] = {"temperature_2m",       "apparent_temperature",
                                                     "relative_humidity_2m", "precipitation",
                                                     "weather_code",         "wind_speed_10m"};
    double values[6] = {};
    for (int i = 0; i < 6; ++i) {
        if (!GetNumber(current, kCurrentFields[i], &values[i])) {
            return std::unexpected(std::string("missing or invalid current.") + kCurrentFields[i]);
        }
    }

    WeatherReport report;
    report.temperature_c = static_cast<float>(values[0]);
    report.feels_like_c = static_cast<float>(values[1]);
    report.humidity_pct = static_cast<int>(values[2]);
    report.precipitation_mm = static_cast<float>(values[3]);
    report.weather_code = static_cast<int>(values[4]);
    report.wind_kmh = static_cast<float>(values[5]);

    auto today = ParseDay(daily, 0);
    if (!today) {
        return std::unexpected(today.error());
    }
    auto tomorrow = ParseDay(daily, 1);
    if (!tomorrow) {
        return std::unexpected(tomorrow.error());
    }
    report.today = *today;
    report.tomorrow = *tomorrow;
    return report;
}

const char* WmoDescription(int code) {
    switch (code) {
        case 0:
            return "clear sky";
        case 1:
            return "mainly clear";
        case 2:
            return "partly cloudy";
        case 3:
            return "overcast";
        case 45:
            return "fog";
        case 48:
            return "depositing rime fog";
        case 51:
            return "light drizzle";
        case 53:
            return "moderate drizzle";
        case 55:
            return "dense drizzle";
        case 56:
            return "light freezing drizzle";
        case 57:
            return "dense freezing drizzle";
        case 61:
            return "slight rain";
        case 63:
            return "moderate rain";
        case 65:
            return "heavy rain";
        case 66:
            return "light freezing rain";
        case 67:
            return "heavy freezing rain";
        case 71:
            return "slight snow fall";
        case 73:
            return "moderate snow fall";
        case 75:
            return "heavy snow fall";
        case 77:
            return "snow grains";
        case 80:
            return "slight rain showers";
        case 81:
            return "moderate rain showers";
        case 82:
            return "violent rain showers";
        case 85:
            return "slight snow showers";
        case 86:
            return "heavy snow showers";
        case 95:
            return "thunderstorm";
        case 96:
            return "thunderstorm with slight hail";
        case 99:
            return "thunderstorm with heavy hail";
        default:
            return nullptr;
    }
}

std::string FormatWeather(const WeatherReport& report, const char* place, int64_t age_s) {
    std::string age = age_s < 60 ? "just now" : std::to_string(age_s / 60) + " minutes ago";
    std::string text;
    if (age_s >= kStaleAfterS) {
        text = "STALE (fetched " + age + ", newer data could not be fetched). ";
    }
    text += "Weather for " + std::string(place) + ", fetched " + age +
            ". Now: " + Condition(report.weather_code) + ", " + Number(report.temperature_c) +
            " C (feels like " + Number(report.feels_like_c) + " C), humidity " +
            std::to_string(report.humidity_pct) + "%, wind " + Number(report.wind_kmh) +
            " km/h, precipitation " + Number(report.precipitation_mm) + " mm. ";
    text += DayText("Today", report.today) + " " + DayText("Tomorrow", report.tomorrow);
    return text;
}

}  // namespace robot_emo

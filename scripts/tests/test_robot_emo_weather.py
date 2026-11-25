"""Host tests for the robot-emo weather parser/formatter (C++23 and the bundled cJSON)."""

import os
import shlex
import shutil
import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BOARD = ROOT / "main/boards/robot-emo"
CJSON = ROOT / "managed_components/espressif__cjson/cJSON"

# A real-shaped Open-Meteo response: current values, today and tomorrow, with
# tomorrow's rain probability null.
FIXTURE = (
    '{"latitude":10.875,"longitude":106.75,"timezone":"Asia/Ho_Chi_Minh",'
    '"current":{"time":"2026-11-25T14:00","interval":900,"temperature_2m":31.4,'
    '"relative_humidity_2m":70,"apparent_temperature":36.2,"precipitation":0.0,'
    '"weather_code":2,"wind_speed_10m":9.7},'
    '"daily":{"time":["2026-11-25","2026-11-26"],"weather_code":[80,95],'
    '"temperature_2m_max":[33.1,32.0],"temperature_2m_min":[25.2,25.0],'
    '"precipitation_probability_max":[60,null]}}'
)

DRIVER = r'''
#include "weather_report.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <string>

using namespace robot_emo;

static bool Near(float a, float b) { return std::fabs(a - b) < 0.01f; }
static bool Has(const std::string& text, const char* part) {
    if (text.find(part) != std::string::npos) return true;
    std::cerr << "missing '" << part << "' in: " << text << '\n';
    return false;
}

int main(int argc, char** argv) {
    std::string body((std::istreambuf_iterator<char>(std::cin)), std::istreambuf_iterator<char>());
    std::string test = argc > 1 ? argv[1] : "";
    if (test == "parses_current_and_two_days") {
        auto r = ParseOpenMeteo(body);
        if (!r) { std::cerr << r.error() << '\n'; return 1; }
        return Near(r->temperature_c, 31.4f) && Near(r->feels_like_c, 36.2f) &&
               r->humidity_pct == 70 && Near(r->precipitation_mm, 0.0f) &&
               r->weather_code == 2 && Near(r->wind_kmh, 9.7f) &&
               r->today.weather_code == 80 && Near(r->today.temp_max_c, 33.1f) &&
               Near(r->today.temp_min_c, 25.2f) && r->today.rain_probability_pct == 60 &&
               r->tomorrow.weather_code == 95 && Near(r->tomorrow.temp_max_c, 32.0f) &&
               Near(r->tomorrow.temp_min_c, 25.0f) ? 0 : 1;
    }
    if (test == "missing_probability_is_unknown") {
        auto r = ParseOpenMeteo(body);
        return r && r->tomorrow.rain_probability_pct == -1 ? 0 : 1;
    }
    if (test == "error") {  // the body must be rejected
        auto r = ParseOpenMeteo(body);
        if (r) return 1;
        std::cout << r.error();
        return 0;
    }
    if (test == "unknown_code_text") {
        auto r = ParseOpenMeteo(body);
        if (!r || WmoDescription(42) != nullptr) return 1;
        r->weather_code = 42;
        return Has(FormatWeather(*r, "Thu Duc City", 300), "unknown (code 42)") ? 0 : 1;
    }
    if (test == "format_contents") {
        auto r = ParseOpenMeteo(body);
        if (!r) return 1;
        std::string text = FormatWeather(*r, "Thu Duc City", 300);
        std::string stale = FormatWeather(*r, "Thu Duc City", 3 * 3600);
        return Has(text, "Thu Duc City") && Has(text, "31.4") && Has(text, "partly cloudy") &&
               Has(text, "Tomorrow") && Has(text, "thunderstorm") &&
               Has(text, "rain chance unknown") && Has(text, "5 minutes ago") &&
               Has(text, "Today (2026-11-25)") && Has(text, "Tomorrow (2026-11-26)") &&
               Has(stale, "STALE") ? 0 : 1;
    }
    std::cerr << "unknown test " << test << '\n';
    return 2;
}
'''


class RobotEmoWeatherTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shlex.split(os.environ.get("CXX", "c++"))
        if not compiler or shutil.which(compiler[0]) is None:
            raise unittest.SkipTest("requires a host C++23 compiler")
        cls.directory = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.directory.cleanup)
        build = Path(cls.directory.name)

        # System cJSON first; else ESP-IDF's already downloaded component (gitignored, so a
        # fresh checkout may not have it). Tests never fetch dependencies.
        cjson_flags = shlex.split(os.environ.get("WEATHER_CJSON_CFLAGS", "-I/usr/include/cjson"))
        cjson_libs = shlex.split(os.environ.get("WEATHER_CJSON_LIBS", "-lcjson"))
        probe = subprocess.run(
            compiler + ["-std=c++23", *cjson_flags, "-x", "c++", "-", *cjson_libs,
                        "-o", str(build / "probe")],
            input="#include <expected>\n#include <cJSON.h>\n"
                  "int main() { std::expected<int, int> v; cJSON_Delete(cJSON_CreateObject()); }",
            text=True, capture_output=True,
        )
        if probe.returncode:
            cc = shlex.split(os.environ.get("CC", "cc"))
            if not (CJSON / "cJSON.c").is_file() or not cc or not shutil.which(cc[0]):
                raise unittest.SkipTest("requires C++23 and cJSON development files: "
                                        + probe.stderr)
            cjson_flags = [f"-I{CJSON}"]
            cjson_object = build / "cjson.o"
            subprocess.run(
                cc + ["-std=c99", *cjson_flags, "-c", str(CJSON / "cJSON.c"),
                      "-o", str(cjson_object)], check=True, capture_output=True, text=True,
            )
            cjson_libs = [str(cjson_object), "-lm"]

        driver = build / "driver.cc"
        driver.write_text(textwrap.dedent(DRIVER), encoding="utf-8")
        cls.executable = build / "weather_test"
        compiled = subprocess.run(
            compiler + ["-std=c++23", f"-I{BOARD}", *cjson_flags, str(driver),
                        str(BOARD / "weather_report.cc"), *cjson_libs,
                        "-o", str(cls.executable)],
            capture_output=True, text=True,
        )
        if compiled.returncode:
            raise RuntimeError("weather host compile failed:\n" + compiled.stderr)

    def run_case(self, case, body=FIXTURE):
        return subprocess.run([str(self.executable), case], input=body, text=True,
                              encoding="utf-8", capture_output=True)

    def assertCase(self, case, body=FIXTURE):
        result = self.run_case(case, body)
        self.assertEqual(result.returncode, 0, result.stderr)
        return result.stdout

    def test_parses_current_and_two_days(self):
        self.assertCase("parses_current_and_two_days")

    def test_missing_probability_is_unknown(self):
        self.assertCase("missing_probability_is_unknown")

    def test_missing_current_is_error(self):
        body = FIXTURE.replace('"temperature_2m":31.4,', "")
        self.assertIn("temperature_2m", self.assertCase("error", body))

    def test_truncated_json_is_error(self):
        self.assertCase("error", FIXTURE[: len(FIXTURE) // 2])

    def test_out_of_range_number_is_error(self):
        # Network input: a value an int cannot hold must be rejected, not cast (UB).
        body = FIXTURE.replace('"weather_code":2,', '"weather_code":1e300,')
        self.assertIn("weather_code", self.assertCase("error", body))

    def test_missing_date_is_error(self):
        body = FIXTURE.replace('"time":["2026-11-25","2026-11-26"],', "")
        self.assertIn("time", self.assertCase("error", body))

    def test_unknown_code_text(self):
        self.assertCase("unknown_code_text")

    def test_format_contents(self):
        self.assertCase("format_contents")


if __name__ == "__main__":
    unittest.main()

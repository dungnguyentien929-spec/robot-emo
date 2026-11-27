"""Host tests for the robot-emo animated-eye renderer (pure C++23, no dependencies)."""

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

DRIVER = r'''
#include "eye_renderer.h"
#include <cstdio>
#include <cstring>
#include <string>

using namespace robot_emo;

static int Lit(const uint8_t* f, int y0 = 0, int y1 = kEyesHeight, int x0 = 0, int x1 = kEyesWidth) {
    int n = 0;
    for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x)
            n += (f[y * kEyesStride + x / 8] >> (7 - x % 8)) & 1;
    return n;
}

// 60 frames, 50 ms apart; returns the time of the last frame.
static uint32_t Converge(EyeRenderer& r, uint8_t* f, uint32_t t = 0) {
    for (int i = 0; i < 60; ++i) { t += 50; r.Render(t, f); }
    return t;
}

// Converges, then returns the largest lit count of a region over 10 more frames, so a
// random blink (140 ms, under 3 frames) cannot make an "open eyes" measurement flaky.
static int OpenLit(EyeRenderer& r, uint8_t* f, uint32_t* t, int y0 = 0, int y1 = kEyesHeight,
                   int x0 = 0, int x1 = kEyesWidth) {
    *t = Converge(r, f, *t);
    int best = 0;
    for (int i = 0; i < 10; ++i) {
        *t += 50;
        r.Render(*t, f);
        int lit = Lit(f, y0, y1, x0, x1);
        best = lit > best ? lit : best;
    }
    return best;
}

int main(int argc, char** argv) {
    std::string test = argc > 1 ? argv[1] : "";
    uint8_t f[kEyesStride * kEyesHeight];

    if (test == "maps_emotions") {
        return ExpressionFromEmotion("funny") == Expression::kLaughing &&
               ExpressionFromEmotion("shocked") == Expression::kSurprised &&
               ExpressionFromEmotion("kissy") == Expression::kLoving &&
               ExpressionFromEmotion("sleepy") == Expression::kSleepy &&
               ExpressionFromEmotion("robot_2") == Expression::kNeutral &&
               ExpressionFromEmotion(nullptr) == Expression::kNeutral ? 0 : 1;
    }
    if (test == "draws_two_eyes") {
        EyeRenderer r(1);
        r.SetMode(EyeMode::kListening);  // no glances, so the eyes stay centred
        uint32_t t = 0;
        int lit = OpenLit(r, f, &t);
        int left = Lit(f, 0, kEyesHeight, 0, 64), right = Lit(f, 0, kEyesHeight, 64, 128);
        printf("lit=%d left=%d right=%d\n", lit, left, right);
        return lit > 600 && lit < 3000 && left > 0 && right > 0 ? 0 : 1;
    }
    if (test == "blink_closes_eyes") {
        EyeRenderer r(1);
        r.SetMode(EyeMode::kListening);
        uint32_t t = 0;
        int open = OpenLit(r, f, &t);
        r.TriggerBlink(t + 10);
        r.Render(t + 30, f);
        int closed = Lit(f);
        printf("open=%d closed=%d\n", open, closed);
        return closed * 4 < open ? 0 : 1;
    }
    if (test == "happy_lights_less_low") {
        EyeRenderer n(1), h(1);
        n.SetMode(EyeMode::kListening);
        h.SetMode(EyeMode::kListening);
        h.SetExpression(Expression::kHappy);
        uint32_t tn = 0, th = 0;
        int neutral_low = OpenLit(n, f, &tn, 40, 64);
        int happy_low = OpenLit(h, f, &th, 40, 64);
        printf("neutral_low=%d happy_low=%d\n", neutral_low, happy_low);
        return happy_low < neutral_low ? 0 : 1;
    }
    if (test == "stays_in_bounds") {
        uint8_t big[64 + sizeof(f) + 64];
        memset(big, 0xA5, sizeof(big));
        uint8_t* frame = big + 64;
        for (int e = 0; e <= static_cast<int>(Expression::kCool); ++e) {
            for (EyeMode m : {EyeMode::kIdle, EyeMode::kListening, EyeMode::kSpeaking}) {
                EyeRenderer r(static_cast<uint32_t>(e) + 7);
                r.SetExpression(static_cast<Expression>(e));
                r.SetMode(m);
                uint32_t t = 0;
                for (int i = 0; i < 200; ++i) { t += 37; r.Render(t, frame); }
            }
        }
        for (int i = 0; i < 64; ++i)
            if (big[i] != 0xA5 || big[64 + sizeof(f) + i] != 0xA5) return 1;
        return 0;
    }
    if (test == "unchanged_frame_not_dirty") {
        EyeRenderer r(1);
        r.SetMode(EyeMode::kListening);
        uint32_t t = Converge(r, f);
        r.Render(t, f);
        return r.Render(t, f) ? 1 : 0;
    }
    fprintf(stderr, "unknown test %s\n", test.c_str());
    return 2;
}
'''


class RobotEmoEyesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shlex.split(os.environ.get("CXX", "c++"))
        if not compiler or shutil.which(compiler[0]) is None:
            raise unittest.SkipTest("requires a host C++23 compiler")
        cls.directory = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.directory.cleanup)
        build = Path(cls.directory.name)
        driver = build / "driver.cc"
        driver.write_text(textwrap.dedent(DRIVER), encoding="utf-8")
        cls.executable = build / "eyes_test"
        compiled = subprocess.run(
            compiler + ["-std=c++23", f"-I{BOARD}", str(driver), str(BOARD / "eye_renderer.cc"),
                        "-o", str(cls.executable)],
            capture_output=True, text=True,
        )
        if compiled.returncode:
            raise RuntimeError("eye renderer host compile failed:\n" + compiled.stderr)

    def check(self, case):
        result = subprocess.run([str(self.executable), case], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_maps_emotions(self):
        self.check("maps_emotions")

    def test_draws_two_eyes(self):
        self.check("draws_two_eyes")

    def test_blink_closes_eyes(self):
        self.check("blink_closes_eyes")

    def test_happy_lights_less_low(self):
        self.check("happy_lights_less_low")

    def test_stays_in_bounds(self):
        self.check("stays_in_bounds")

    def test_unchanged_frame_not_dirty(self):
        self.check("unchanged_frame_not_dirty")


if __name__ == "__main__":
    unittest.main()

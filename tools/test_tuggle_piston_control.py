#!/usr/bin/env python3
"""Source-level contract checks for Down mode and independent A/Y controls."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADER = (ROOT / "include/motorcontrol.h").read_text()
IMPLEMENTATION = (ROOT / "src/motorcontrol.cpp").read_text()
MAIN = (ROOT / "src/main.cpp").read_text()


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    opening_brace = source.index("{", start)
    depth = 0
    for index in range(opening_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening_brace + 1:index]
    raise AssertionError("function body is not closed")


expected_signature = "void TugglePistonControl(bool BtnA, bool BtnY, bool BtnDown, bool Reset)"
assert expected_signature[:-1] + " = false);" in HEADER
assert expected_signature in IMPLEMENTATION

body = function_body(IMPLEMENTATION, expected_signature)
assert "static bool extendedMode = false;" in body
assert "if (!prevDown && BtnDown)" in body
assert "extendedMode = !extendedMode;" in body
assert "prevDown = BtnDown;" in body
assert "Piston_tuggle.set_value(extendedMode || BtnY);" in body
assert "Piston_tuggle2.set_value(extendedMode || BtnA);" in body
reset_body = function_body(body, "if (Reset)")
assert "extendedMode = false;" in reset_body
assert "prevDown = BtnDown;" in reset_body
assert "Piston_tuggle.set_value(false);" in reset_body
assert "Piston_tuggle2.set_value(false);" in reset_body
assert "return;" in reset_body
assert "pros::delay" not in body
assert "TugglePistonControl(BtnA, BtnY, BtnDown);" in MAIN
assert "TugglePistonControl(false, false, master.get_digital(DIGITAL_DOWN), true);" in MAIN
assert "int BtnDown = master.get_digital(DIGITAL_DOWN);" in MAIN
assert "ChassisLock(BtnUp)" in MAIN
print("PASS: Down toggle, A/Y mapping, phase reset, and Up chassis lock source contracts")

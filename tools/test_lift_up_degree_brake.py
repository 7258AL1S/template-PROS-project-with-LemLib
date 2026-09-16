#!/usr/bin/env python3
"""Source-level contract checks for LiftUpDegree's stopping sequence."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
IMPLEMENTATION = (ROOT / "src/motorcontrol.cpp").read_text()


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


body = function_body(IMPLEMENTATION, "void LiftUpDegree(float Power, float Target, float Fulltime)")
assert "constexpr uint32_t kStopBrakeMs = 150;" in body

stop_start = body.index("auto brakeThenHold = []()")
stop_body = function_body(body[stop_start:], "auto brakeThenHold = []()")

brake_mode = "pros::E_MOTOR_BRAKE_BRAKE"
hold_mode = "pros::E_MOTOR_BRAKE_HOLD"
delay = "pros::delay(kStopBrakeMs);"
assert stop_body.index(brake_mode) < stop_body.index(delay) < stop_body.index(hold_mode)
assert stop_body.count("lift1.brake();") == 2
assert stop_body.count("lift2.brake();") == 2

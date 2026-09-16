#!/usr/bin/env python3
"""Source-level contract checks for the independent tuggle piston controls."""

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


expected_signature = "void TugglePistonControl(bool BtnA, bool BtnY)"
assert expected_signature in HEADER
assert expected_signature in IMPLEMENTATION

body = function_body(IMPLEMENTATION, expected_signature)
assert "Piston_tuggle.set_value(BtnA);" in body
assert "Piston_tuggle2.set_value(BtnY);" in body
assert "pistonActive" not in body
assert "prevA" not in body
assert "TugglePistonControl(BtnA, BtnY);" in MAIN

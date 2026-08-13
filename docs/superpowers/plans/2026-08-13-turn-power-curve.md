# turnTo 电机功率曲线工具 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 创建一个可运行的 Python 脚本，输入 LemLib `turnTo` 的 PID 三个值，输出 `timeout`（FullTime）内的 `motorPower` 曲线。

**Architecture:** 单文件工具 `tools/turn_power_curve.py`，用固定步长循环模拟 `turnTo` 的角误差、PID、slew、constrainPower 和退出条件；matplotlib 负责绘图。另加一个 unittest 测试文件验证核心函数。

**Tech Stack:** Python 3.12，matplotlib 3.11，标准库 `argparse` / `unittest` / `math`。

**解释器:** `E:\conda\envs\pytorch\python.exe`

---

## File Structure

- Create: `tools/turn_power_curve.py` — 仿真核心、CLI、绘图。
- Create: `tools/test_turn_power_curve.py` — 核心函数的 unittest。
- Modify: 无现有源码修改。

---

## Task 1: 编写失败测试

**Files:**
- Create: `tools/test_turn_power_curve.py`

- [ ] **Step 1: 创建测试文件**

```python
import importlib.util
import math
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parent / "turn_power_curve.py"
spec = importlib.util.spec_from_file_location("turn_power_curve", MODULE_PATH)
turn = importlib.util.module_from_spec(spec)
spec.loader.exec_module(turn)


class WrapAngleErrorTests(unittest.TestCase):
    def test_direct_negative(self):
        self.assertAlmostEqual(turn.wrap_angle_error_deg(-90.0, 0.0), -90.0)

    def test_shortest_positive(self):
        self.assertAlmostEqual(turn.wrap_angle_error_deg(10.0, 350.0), 20.0)

    def test_shortest_negative(self):
        self.assertAlmostEqual(turn.wrap_angle_error_deg(350.0, 10.0), -20.0)


class ConstrainPowerTests(unittest.TestCase):
    def test_within_range(self):
        self.assertAlmostEqual(turn.constrain_power(0.5, 1.0, 0.0), 0.5)

    def test_zero_min_speed(self):
        self.assertAlmostEqual(turn.constrain_power(0.0, 1.0, 0.2), 0.0)

    def test_min_speed_negative(self):
        self.assertAlmostEqual(turn.constrain_power(-0.1, 1.0, 0.2), -0.2)

    def test_max_speed_clamp(self):
        self.assertAlmostEqual(turn.constrain_power(2.0, 1.0, 0.0), 1.0)


class PIDTests(unittest.TestCase):
    def test_first_update_is_proportional(self):
        pid = turn.LemLibPID(kp=2.0, ki=0.0, kd=0.0)
        self.assertAlmostEqual(pid.update(0.5, 0.01), 1.0)

    def test_integral_accumulates(self):
        pid = turn.LemLibPID(kp=0.0, ki=1.0, kd=0.0)
        pid.update(1.0, 0.01)
        self.assertAlmostEqual(pid.update(1.0, 0.01), 0.01)

    def test_derivative_is_used(self):
        pid = turn.LemLibPID(kp=0.0, ki=0.0, kd=1.0)
        pid.update(0.0, 0.01)
        self.assertAlmostEqual(pid.update(0.1, 0.01), 10.0)


class SlewPowerTests(unittest.TestCase):
    def test_increasing_limit(self):
        self.assertAlmostEqual(turn.slew_power(1.0, 0.0, 2.0, 0.01, "increasing"), 0.02)

    def test_decreasing_limit(self):
        self.assertAlmostEqual(turn.slew_power(-1.0, 0.0, 2.0, 0.01, "decreasing"), -0.02)

    def test_unrestricted_opposite_direction(self):
        self.assertAlmostEqual(turn.slew_power(1.0, 0.0, 2.0, 0.01, "decreasing"), 1.0)


class SimulateTurnPowerTests(unittest.TestCase):
    def test_default_curve_length_and_bounds(self):
        times, powers = turn.simulate_turn_power(
            kp=1.32,
            ki=0.0,
            kd=0.1,
            initial_deg=0.0,
            target_deg=-90.0,
            fulltime_ms=900.0,
            dt_ms=10.0,
        )
        self.assertEqual(len(times), 91)
        self.assertEqual(times[0], 0.0)
        self.assertEqual(times[-1], 900.0)
        self.assertTrue(all(-1.0 <= p <= 1.0 for p in powers))
        self.assertAlmostEqual(powers[-1], 0.0)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行测试，确认失败**

Run: `E:\conda\envs\pytorch\python.exe tools\test_turn_power_curve.py`

Expected: FAIL，报错 `ModuleNotFoundError` 或 `AttributeError`，因为 `tools/turn_power_curve.py` 还不存在。

- [ ] **Step 3: 提交测试文件**

```bash
git add tools/test_turn_power_curve.py
git commit -m "test: turnTo 电机功率曲线工具核心函数测试"
```

---

## Task 2: 实现核心脚本

**Files:**
- Create: `tools/turn_power_curve.py`

- [ ] **Step 1: 创建脚本，包含核心函数**

```python
from __future__ import annotations

import argparse
import math

import matplotlib.pyplot as plt


def sign(x: float) -> float:
    """复刻 LemLib units::sgn 对 Number 的行为。"""
    if x > 0:
        return 1.0
    if x < 0:
        return -1.0
    return 0.0


def wrap_angle_error_deg(target: float, current: float) -> float:
    """AUTO 模式最短角误差，范围为 (-180, 180] 度。"""
    target = (target % 360.0 + 360.0) % 360.0
    error = target - current
    error = math.remainder(error, 360.0)
    return error


def constrain_power(power: float, max_speed: float, min_speed: float) -> float:
    """复刻 LemLib constrainPower。"""
    if min_speed != 0 and abs(power) < min_speed:
        power = sign(power) * min_speed
    return max(-max_speed, min(max_speed, power))


def slew_power(
    target: float,
    current: float,
    max_rate: float,
    dt: float,
    restrict_direction: str,
) -> float:
    """复刻 LemLib slew 的单方向限制逻辑。"""
    if max_rate == 0:
        return target

    change = target - current
    if restrict_direction == "increasing" and change < 0:
        return target
    if restrict_direction == "decreasing" and change > 0:
        return target

    limit = max_rate * dt
    if abs(change) > abs(limit):
        return current + limit * sign(change)
    return target


class LemLibPID:
    """按固定 dt 复刻 LemLib PID::update 的数值行为。"""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        windup_range: float = 0.0,
        sign_flip_reset: bool = False,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.windup_range = windup_range
        self.sign_flip_reset = sign_flip_reset
        self.previous_error: float | None = None
        self.integral = 0.0

    def update(self, error: float, dt: float) -> float:
        if self.previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.previous_error) / dt

        self.integral += error * dt

        if self.sign_flip_reset and self.previous_error is not None:
            if sign(error) != sign(self.previous_error):
                self.integral = 0.0

        if self.windup_range != 0 and abs(error) > self.windup_range:
            self.integral = 0.0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


def simulate_turn_power(
    *,
    kp: float,
    ki: float,
    kd: float,
    initial_deg: float = 0.0,
    target_deg: float = -90.0,
    fulltime_ms: float = 900.0,
    dt_ms: float = 10.0,
    max_speed: float = 1.0,
    min_speed: float = 0.0,
    slew: float = 0.0,
    max_omega_deg_per_s: float = 360.0,
    exit_range_deg: float = 2.0,
    exit_time_ms: float = 250.0,
    early_exit_range_deg: float = 0.0,
    windup_range: float = 0.0,
    sign_flip_reset: bool = False,
) -> tuple[list[float], list[float]]:
    """模拟 turnTo，返回 (time_ms, motor_power) 两个等长列表。"""
    steps = max(1, int(math.floor(fulltime_ms / dt_ms)))
    times = [i * dt_ms for i in range(steps)]
    times.append(fulltime_ms)

    dt = dt_ms / 1000.0
    heading = initial_deg
    pid = LemLibPID(kp, ki, kd, windup_range, sign_flip_reset)

    prev_raw: float | None = None
    prev_delta: float | None = None
    prev_power = 0.0
    initial_error = wrap_angle_error_deg(target_deg, initial_deg)
    slew_direction = "increasing" if initial_error > 0 else "decreasing"

    exit_start: float | None = None
    exited = False
    powers: list[float] = []

    for t in times[:-1]:
        if exited:
            powers.append(0.0)
            continue

        raw = wrap_angle_error_deg(target_deg, heading)
        settling = prev_raw is not None and sign(raw) != sign(prev_raw)
        prev_raw = raw

        error = raw
        if prev_delta is None:
            prev_delta = error

        if min_speed != 0:
            if abs(error) < early_exit_range_deg:
                exited = True
                powers.append(0.0)
                continue
            if sign(error) != sign(prev_delta):
                exited = True
                powers.append(0.0)
                continue

        prev_delta = error

        raw_power = pid.update(math.radians(error), dt)
        if not settling:
            raw_power = slew_power(raw_power, prev_power, slew, dt, slew_direction)
        power = constrain_power(raw_power, max_speed, min_speed)
        prev_power = power
        powers.append(power)

        if exit_start is None:
            exit_start = t
        if abs(error) >= exit_range_deg:
            exit_start = None
        elif t - exit_start >= exit_time_ms:
            exited = True

        heading += power * max_omega_deg_per_s * dt

    powers.append(0.0)
    return times, powers
```

- [ ] **Step 2: 运行测试，确认核心函数通过**

Run: `E:\conda\envs\pytorch\python.exe tools\test_turn_power_curve.py`

Expected: PASS，所有测试通过。

- [ ] **Step 3: 添加 CLI、绘图入口和 main**

将以下代码追加到 `tools/turn_power_curve.py` 末尾：

```python
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Simulate lemlib::turnTo and plot motor power vs time."
    )
    parser.add_argument("kp", type=float, nargs="?", default=1.32, help="P gain")
    parser.add_argument("ki", type=float, nargs="?", default=0.0, help="I gain")
    parser.add_argument("kd", type=float, nargs="?", default=0.1, help="D gain")
    parser.add_argument("--initial", type=float, default=0.0, help="initial heading, deg")
    parser.add_argument("--target", type=float, default=-90.0, help="target heading, deg")
    parser.add_argument("--fulltime", type=float, default=900.0, help="timeout, ms")
    parser.add_argument("--dt", type=float, default=10.0, help="simulation step, ms")
    parser.add_argument("--max-speed", type=float, default=1.0, help="max motor power")
    parser.add_argument("--min-speed", type=float, default=0.0, help="min motor power")
    parser.add_argument("--slew", type=float, default=0.0, help="max power change per second")
    parser.add_argument("--max-omega", type=float, default=360.0, help="max angular speed, deg/s")
    parser.add_argument("--exit-range", type=float, default=2.0, help="exit error range, deg")
    parser.add_argument("--exit-time", type=float, default=250.0, help="exit settle time, ms")
    parser.add_argument("--early-exit-range", type=float, default=0.0, help="early exit range, deg")
    parser.add_argument("--windup-range", type=float, default=0.0, help="PID integral windup range")
    parser.add_argument("--sign-flip-reset", action="store_true", help="reset integral on error sign flip")
    parser.add_argument("--save", type=str, default=None, help="save PNG to this path")
    return parser


def validate_args(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    if args.fulltime <= 0:
        parser.error("--fulltime must be positive")
    if args.dt <= 0:
        parser.error("--dt must be positive")
    if args.max_speed < 0:
        parser.error("--max-speed must be >= 0")
    if args.min_speed < 0:
        parser.error("--min-speed must be >= 0")
    if args.min_speed > args.max_speed:
        parser.error("--min-speed must be <= --max-speed")
    if args.slew < 0:
        parser.error("--slew must be >= 0")
    if args.max_omega <= 0:
        parser.error("--max-omega must be positive")
    if args.exit_range < 0:
        parser.error("--exit-range must be >= 0")
    if args.exit_time < 0:
        parser.error("--exit-time must be >= 0")
    if args.early_exit_range < 0:
        parser.error("--early-exit-range must be >= 0")
    if args.windup_range < 0:
        parser.error("--windup-range must be >= 0")


def plot_power_curve(
    times_ms: list[float],
    powers: list[float],
    args: argparse.Namespace,
) -> None:
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times_ms, powers, label="motorPower", linewidth=2)
    ax.axhline(0.0, color="gray", linewidth=0.8)
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Motor power")
    ax.set_title(
        f"turnTo motor power: kP={args.kp}, kI={args.ki}, kD={args.kd}\n"
        f"initial={args.initial} deg, target={args.target} deg, fulltime={args.fulltime} ms"
    )
    ax.grid(True)

    peak = max(powers)
    trough = min(powers)
    peak_index = powers.index(peak)
    trough_index = powers.index(trough)
    ax.annotate(
        f"peak {peak:.3f}",
        xy=(times_ms[peak_index], peak),
        xytext=(times_ms[peak_index] + 25, peak + 0.08),
        arrowprops={"arrowstyle": "->"},
    )
    ax.annotate(
        f"trough {trough:.3f}",
        xy=(times_ms[trough_index], trough),
        xytext=(times_ms[trough_index] + 25, trough - 0.08),
        arrowprops={"arrowstyle": "->"},
    )
    ax.legend()

    if args.save:
        fig.savefig(args.save, dpi=150)
        print(f"Saved plot to {args.save}")
    plt.show()


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(args, parser)

    times_ms, powers = simulate_turn_power(
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        initial_deg=args.initial,
        target_deg=args.target,
        fulltime_ms=args.fulltime,
        dt_ms=args.dt,
        max_speed=args.max_speed,
        min_speed=args.min_speed,
        slew=args.slew,
        max_omega_deg_per_s=args.max_omega,
        exit_range_deg=args.exit_range,
        exit_time_ms=args.exit_time,
        early_exit_range_deg=args.early_exit_range,
        windup_range=args.windup_range,
        sign_flip_reset=args.sign_flip_reset,
    )
    plot_power_curve(times_ms, powers, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: 运行测试，确认完整脚本仍通过**

Run: `E:\conda\envs\pytorch\python.exe tools\test_turn_power_curve.py`

Expected: PASS。

- [ ] **Step 5: 提交脚本**

```bash
git add tools/turn_power_curve.py
git commit -m "feat: 新增 turnTo 电机功率曲线模拟工具"
```

---

## Task 3: 用第 106 行场景做端到端验证

**Files:**
- 无新增文件。

- [ ] **Step 1: 用默认参数运行，检查输出范围与形状**

Run:

```text
E:\conda\envs\pytorch\python.exe tools\turn_power_curve.py 1.32 0.0 0.1
```

Expected:
- matplotlib 窗口弹出。
- 初始功率为 `-1.0`（因为目标 -90°，误差 -90°，P 输出约 -2.07，被 `maxSpeed=1` 钳位到 -1.0）。
- 曲线接近 0 后因退出条件变为 0。
- 所有功率都在 `[-1, 1]`。

- [ ] **Step 2: 保存 PNG 做记录**

Run:

```text
E:\conda\envs\pytorch\python.exe tools\turn_power_curve.py 1.32 0.0 0.1 --save docs\superpowers\plans\turn-power-curve-line106.png
```

Expected: 命令结束后存在 `docs/superpowers/plans/turn-power-curve-line106.png`。

- [ ] **Step 3: 提交验证产物**

```bash
git add docs/superpowers/plans/turn-power-curve-line106.png
git commit -m "test: 记录第106行 turnTo PID 的电机功率曲线"
```

- [ ] **Step 4: 推送所有提交**

```bash
git push origin main
```

---

## Self-Review

**Spec coverage:**
- 单文件 `tools/turn_power_curve.py`：Task 2。
- 全部参数可调：Task 2 Step 4 `build_parser`。
- 复刻 LemLib PID/angleError/slew/constrainPower/退出条件：Task 2 Step 1。
- matplotlib 绘图和 `--save`：Task 2 Step 4。
- 第 106 行默认测试：Task 3。

**Placeholder scan:** 无 TBD/TODO；Task 2 Step 3 的草稿被 Step 4 明确替换。

**Type consistency:** `simulate_turn_power` 返回 `tuple[list[float], list[float]]`，`plot_power_curve` 接收同名类型；测试中的参数名与函数签名一致。

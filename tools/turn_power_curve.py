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
    """AUTO 模式最短角误差，范围为 [-180, 180) 度。"""
    target = (target % 360.0 + 360.0) % 360.0
    error = target - current
    return math.remainder(error, 360.0)


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
            # LemLib 首次 update 时 previousTime 尚未设置，dt 视为 0。
            dt = 0.0
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

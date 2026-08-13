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

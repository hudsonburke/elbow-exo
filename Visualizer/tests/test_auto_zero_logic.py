import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PID_vis import update_auto_zero_state


class AutoZeroLogicTests(unittest.TestCase):
    def test_auto_zero_triggers_after_timeout_without_enough_angle_change(self) -> None:
        state = {}

        should_zero, state = update_auto_zero_state(
            state,
            current_angle=10.0,
            pwm_cmd=50.0,
            now_s=100.0,
            angle_threshold_deg=5.0,
            time_threshold_s=3.0,
            pwm_threshold=5.0,
        )
        self.assertFalse(should_zero)

        should_zero, state = update_auto_zero_state(
            state,
            current_angle=10.0,
            pwm_cmd=50.0,
            now_s=103.0,
            angle_threshold_deg=5.0,
            time_threshold_s=3.0,
            pwm_threshold=5.0,
        )
        self.assertTrue(should_zero)

    def test_auto_zero_resets_when_angle_moves_enough(self) -> None:
        state = {}

        should_zero, state = update_auto_zero_state(
            state,
            current_angle=10.0,
            pwm_cmd=50.0,
            now_s=100.0,
            angle_threshold_deg=5.0,
            time_threshold_s=3.0,
            pwm_threshold=5.0,
        )
        self.assertFalse(should_zero)

        should_zero, state = update_auto_zero_state(
            state,
            current_angle=16.0,
            pwm_cmd=50.0,
            now_s=101.0,
            angle_threshold_deg=5.0,
            time_threshold_s=3.0,
            pwm_threshold=5.0,
        )
        self.assertFalse(should_zero)
        self.assertFalse(state["tracking"])


if __name__ == "__main__":
    unittest.main()

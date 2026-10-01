# Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
"""Meaningful controller checks runnable with unittest without ROS installed."""

from dataclasses import replace
import math
import unittest

from team_chase_object.control import (
    ChaseConfig, ChaseController, PID, PIDConfig, PolarObservation, VelocityCommand,
)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = ChaseController()
        self.observation = PolarObservation(True, 0.0, 1.0, 0.9, 10.0)

    def command(self, **changes):
        observation = replace(self.observation, **changes)
        self.assertTrue(self.controller.receive(observation, 10.0, 20.0))
        return self.controller.step(10.0, 20.0)

    def test_startup_without_observation_is_stopped(self):
        self.assertEqual(self.controller.step(10.0, 20.0), VelocityCommand())

    def test_forward_reverse_and_turn_signs(self):
        self.assertGreater(self.command(distance=1.0, bearing=0.2).linear_x, 0.0)
        self.controller = ChaseController()
        command = self.command(distance=0.3, bearing=-0.2)
        self.assertLess(command.linear_x, 0.0)
        self.assertLess(command.angular_z, 0.0)
        self.controller = ChaseController()
        self.assertGreater(self.command(bearing=0.2).angular_z, 0.0)

    def test_limits_and_heading_gate(self):
        command = self.command(distance=10.0, bearing=0.5)
        self.assertEqual(command.linear_x, 0.15)
        self.assertLessEqual(command.angular_z, 0.8)
        self.controller = ChaseController()
        self.assertEqual(self.command(distance=0.01).linear_x, -0.1)
        self.controller = ChaseController()
        command = self.command(distance=10.0, bearing=1.0)
        self.assertEqual(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.8)

    def test_deadbands_are_exactly_zero_and_clear_integral(self):
        config = ChaseConfig(linear_pid=PIDConfig(0.2, 0.5), angular_pid=PIDConfig(1.0, 0.5))
        self.controller = ChaseController(config)
        self.command(distance=0.7, bearing=0.1)
        self.assertNotEqual(self.controller.linear.integral, 0.0)
        self.assertNotEqual(self.controller.angular.integral, 0.0)
        observation = replace(self.observation, distance=0.62, bearing=0.01, stamp=10.1)
        self.assertTrue(self.controller.receive(observation, 10.1, 20.1))
        self.assertEqual(self.controller.step(10.1, 20.1), VelocityCommand())
        self.assertEqual(self.controller.linear.integral, 0.0)
        self.assertEqual(self.controller.angular.integral, 0.0)

    def test_deadband_boundary_remains_zero_with_float_roundoff(self):
        self.assertEqual(self.command(distance=0.65, bearing=0.04), VelocityCommand())

    def test_each_invalid_observation_stops_and_forgets_target(self):
        invalid_cases = (
            {"detected": False}, {"distance": 0.0}, {"distance": -1.0},
            {"distance": math.inf}, {"bearing": math.nan}, {"bearing": math.pi + 0.01},
            {"confidence": math.nan}, {"confidence": 0.1}, {"confidence": 1.1},
            {"frame_id": "laser"}, {"stamp": 0.0}, {"stamp": math.nan},
            {"stamp": 9.4}, {"stamp": 10.11},
        )
        for changes in invalid_cases:
            with self.subTest(changes=changes):
                self.controller = ChaseController()
                self.assertNotEqual(self.command(), VelocityCommand())
                invalid = replace(self.observation, stamp=10.01)
                self.assertFalse(self.controller.receive(replace(invalid, **changes), 10.0, 20.1))
                self.assertEqual(self.controller.step(10.0, 20.1), VelocityCommand())
                self.assertEqual(self.controller.linear.integral, 0.0)
                self.assertIsNone(self.controller.angular.previous_error)

    def test_receipt_watchdog_works_when_simulated_time_is_paused(self):
        self.command()
        self.assertEqual(self.controller.step(10.0, 20.51), VelocityCommand())
        self.assertEqual(self.controller.last_stop_reason, "receive watchdog expired")

    def test_source_age_expires_before_newer_receipt_time(self):
        self.command()
        self.assertTrue(self.controller.receive(replace(self.observation, stamp=10.01), 10.4, 20.4))
        self.assertEqual(self.controller.step(10.52, 20.5), VelocityCommand())
        self.assertEqual(self.controller.last_stop_reason, "source observation is stale")

    def test_duplicate_replay_cannot_refresh_receipt_when_ros_time_pauses(self):
        self.command()
        for steady_time in (20.1, 20.4, 20.8, 21.0):
            self.assertFalse(self.controller.receive(self.observation, 10.0, steady_time))
            self.assertEqual(self.controller.step(10.0, steady_time), VelocityCommand())
        self.assertEqual(self.controller.last_stop_reason, "duplicate or out-of-order source timestamp")

    def test_older_positive_after_newer_lost_cannot_restart(self):
        self.command()
        lost = replace(self.observation, detected=False, stamp=10.2)
        self.assertFalse(self.controller.receive(lost, 10.2, 20.2))
        self.controller.reset()
        delayed = replace(self.observation, stamp=10.1)
        self.assertFalse(self.controller.receive(delayed, 10.3, 20.3))
        self.assertEqual(self.controller.step(10.3, 20.3), VelocityCommand())
        recovered = replace(self.observation, stamp=10.4)
        self.assertTrue(self.controller.receive(recovered, 10.4, 20.4))
        self.assertGreater(self.controller.step(10.4, 20.4).linear_x, 0.0)

    def test_invalid_payload_also_prevents_older_positive_replay(self):
        self.command()
        invalid = replace(self.observation, distance=math.nan, stamp=10.2)
        self.assertFalse(self.controller.receive(invalid, 10.2, 20.2))
        self.assertFalse(self.controller.receive(replace(self.observation, stamp=10.1), 10.3, 20.3))
        self.assertEqual(self.controller.step(10.3, 20.3), VelocityCommand())

    def test_backward_source_stamp_alone_does_not_clear_history(self):
        self.command()
        older = replace(self.observation, stamp=9.9)
        self.assertFalse(self.controller.receive(older, 10.1, 20.1))
        self.assertEqual(self.controller.step(10.1, 20.1), VelocityCommand())

    def test_actual_ros_clock_reset_allows_new_epoch_after_stopping(self):
        self.command()
        self.assertEqual(self.controller.step(1.0, 20.1), VelocityCommand())
        new_epoch = replace(self.observation, stamp=1.1)
        self.assertTrue(self.controller.receive(new_epoch, 1.1, 20.2))
        self.assertGreater(self.controller.step(1.1, 20.2).linear_x, 0.0)

    def test_receive_can_be_first_callback_after_ros_clock_reset(self):
        self.command()
        new_epoch = replace(self.observation, stamp=1.0)
        self.assertTrue(self.controller.receive(new_epoch, 1.0, 20.1))
        self.assertGreater(self.controller.step(1.0, 20.1).linear_x, 0.0)

    def test_clock_jump_and_nonfinite_clock_stop(self):
        for now_ros, now_monotonic in ((9.0, 20.1), (10.0, 19.0), (math.nan, 20.1), (10.0, math.inf)):
            with self.subTest(now_ros=now_ros, now_monotonic=now_monotonic):
                self.controller = ChaseController()
                self.command()
                self.assertEqual(self.controller.step(now_ros, now_monotonic), VelocityCommand())

    def test_lost_target_resets_derivative_before_reacquisition(self):
        self.controller = ChaseController(ChaseConfig(angular_pid=PIDConfig(1.0, 0.0, 0.5)))
        self.command(bearing=0.3)
        self.controller.receive(replace(self.observation, detected=False), 10.1, 20.1)
        self.assertTrue(self.controller.receive(replace(self.observation, bearing=-0.1, stamp=10.2), 10.2, 20.2))
        self.assertAlmostEqual(self.controller.step(10.2, 20.2).angular_z, -0.1)

    def test_invalid_config_fails_fast(self):
        invalid = (
            {"target_distance": 0.0}, {"observation_timeout": 0.0},
            {"control_rate_hz": math.nan}, {"heading_gate": math.inf},
            {"bearing_deadband": 0.7}, {"distance_deadband": 0.6},
            {"min_confidence": 1.1}, {"max_future_stamp": 0.11}, {"base_frame": ""},
            {"max_forward_speed": -0.1},
        )
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                ChaseConfig(**values)
        for gain in (-1.0, math.nan, math.inf):
            with self.subTest(gain=gain), self.assertRaises(ValueError):
                PIDConfig(kp=gain)


class PIDTests(unittest.TestCase):
    def test_saturation_does_not_wind_up_integral(self):
        pid = PID(PIDConfig(1.0, 1.0))
        for _ in range(100):
            self.assertEqual(pid.step(10.0, 0.1, -1.0, 1.0), 1.0)
        self.assertEqual(pid.integral, 0.0)
        self.assertLess(pid.step(-0.1, 0.1, -1.0, 1.0), 0.0)

    def test_pid_variants_and_derivative_reset(self):
        proportional = PID(PIDConfig(2.0))
        self.assertEqual(proportional.step(0.2, 0.1, -10.0, 10.0), 0.4)
        integral = PID(PIDConfig(0.0, 1.0))
        self.assertAlmostEqual(integral.step(0.2, 0.5, -10.0, 10.0), 0.1)
        derivative = PID(PIDConfig(0.0, 0.0, 1.0))
        self.assertEqual(derivative.step(0.2, 0.1, -10.0, 10.0), 0.0)
        self.assertAlmostEqual(derivative.step(0.3, 0.1, -10.0, 10.0), 1.0)
        derivative.reset()
        self.assertEqual(derivative.step(-0.5, 0.1, -10.0, 10.0), 0.0)


if __name__ == "__main__":
    unittest.main()

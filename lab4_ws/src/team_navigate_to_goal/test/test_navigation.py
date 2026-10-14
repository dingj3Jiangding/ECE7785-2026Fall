"""Behavior checks for navigation without importing ROS or moving a robot.

Lab partners: Ding Jiang, Tongning Zhang.
"""

import math
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
TEST_ROOT = Path(__file__).resolve().parent
for directory in (PACKAGE_ROOT, TEST_ROOT):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from team_navigate_to_goal.geometry import Pose2D  # noqa: E402
from team_navigate_to_goal.navigation import (  # noqa: E402
    NavigationConfig,
    Navigator,
    Waypoint,
)
from simulation import ideal_scan, run_scenario  # noqa: E402


class NavigationBehaviorTests(unittest.TestCase):
    def setUp(self):
        self.origin = Pose2D(0.0, 0.0, 0.0)
        self.empty_scan = ideal_scan(self.origin)

    def assert_stopped(self, command):
        self.assertEqual(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)

    def test_heading_wrap_takes_short_turn_across_pi(self):
        navigator = Navigator([Waypoint(-1.0, -0.01, 0.05)])
        command = navigator.step(Pose2D(0.0, 0.0, math.pi - 0.01), self.empty_scan, 0.0)
        self.assertGreater(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)
        self.assertLess(command.angular_z, 0.1)

    def test_large_heading_error_turns_before_driving(self):
        navigator = Navigator([Waypoint(0.0, 1.0, 0.05)])
        command = navigator.step(self.origin, self.empty_scan, 0.0)
        self.assertEqual(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)
        self.assertLessEqual(command.angular_z, NavigationConfig().max_angular_speed)

    def test_forward_command_obeys_speed_limit(self):
        navigator = Navigator([Waypoint(100.0, 0.0, 0.05)])
        command = navigator.step(self.origin, self.empty_scan, 0.0)
        self.assertGreater(command.linear_x, 0.0)
        self.assertLessEqual(command.linear_x, NavigationConfig().max_forward_speed)
        self.assertEqual(command.angular_z, 0.0)

    def test_waypoints_are_visited_in_file_order(self):
        navigator = Navigator([Waypoint(1.5, 0.0, 0.10), Waypoint(0.0, 0.0, 0.20)])
        command = navigator.step(self.origin, self.empty_scan, 0.0)
        self.assertEqual(navigator.goal_index, 0)
        self.assertNotEqual(navigator.state, "HOLD")
        self.assertGreater(command.linear_x, 0.0)

    def test_each_waypoint_holds_ten_seconds_then_final_stops(self):
        navigator = Navigator([Waypoint(0.0, 0.0, 0.10), Waypoint(1.5, 0.0, 0.10)])
        self.assert_stopped(navigator.step(self.origin, self.empty_scan, 0.0))
        self.assertEqual(navigator.state, "HOLD")
        self.assert_stopped(navigator.step(self.origin, self.empty_scan, 9.99))
        self.assertEqual(navigator.goal_index, 0)
        navigator.step(self.origin, self.empty_scan, 10.01)
        self.assertEqual(navigator.goal_index, 1)
        self.assertNotEqual(navigator.state, "DONE")

        second_goal = Pose2D(1.5, 0.0, 0.0)
        self.assert_stopped(navigator.step(second_goal, self.empty_scan, 11.0))
        self.assertEqual(navigator.state, "HOLD")
        self.assert_stopped(navigator.step(second_goal, self.empty_scan, 20.99))
        self.assertNotEqual(navigator.state, "DONE")
        self.assert_stopped(navigator.step(second_goal, self.empty_scan, 21.01))
        self.assertEqual(navigator.state, "DONE")
        self.assert_stopped(navigator.step(second_goal, self.empty_scan, 200.0))

    def test_sensor_pause_requires_a_new_complete_goal_hold(self):
        navigator = Navigator([Waypoint(0.0, 0.0, 0.10)])
        navigator.step(self.origin, self.empty_scan, 0.0)
        navigator.step(self.origin, self.empty_scan, 5.0)
        navigator.pause("scan timeout")
        self.assert_stopped(navigator.step(self.origin, self.empty_scan, 100.0))
        self.assertEqual(navigator.state, "HOLD")
        self.assert_stopped(navigator.step(self.origin, self.empty_scan, 109.99))
        self.assertNotEqual(navigator.state, "DONE")
        self.assert_stopped(navigator.step(self.origin, self.empty_scan, 110.01))
        self.assertEqual(navigator.state, "DONE")


class FullRouteSimulationTests(unittest.TestCase):
    def assert_completed_route(self, result):
        details = str(result.summary())
        self.assertFalse(result.collision, details)
        self.assertTrue(result.done, details)
        self.assertLessEqual(result.elapsed, 150.0, details)
        self.assertEqual(len(result.hold_durations), 3, details)
        for duration in result.hold_durations:
            self.assertGreaterEqual(duration + 1e-9, 10.0, details)
        for error, tolerance in zip(result.arrival_errors, (0.10, 0.15, 0.20)):
            self.assertLessEqual(error, tolerance, details)

    def test_empty_route(self):
        self.assert_completed_route(run_scenario())

    def test_fixed_and_later_inserted_rotated_boxes(self):
        for angle in (0.0, 30.0, 60.0, 90.0):
            with self.subTest(angle_degrees=angle):
                result = run_scenario(with_obstacles=True, obstacle_angle_degrees=angle)
                self.assert_completed_route(result)
                self.assertTrue(result.inserted_obstacle, str(result.summary()))

    def test_rotated_boxes_after_a_different_goal_stopping_position(self):
        # Changing the second stopping position exposes competition between the
        # previous side obstacle and the newly inserted front obstacle.
        result = run_scenario(with_obstacles=True, obstacle_angle_degrees=60.0,
                              tolerances=(0.10, 0.15, 0.20))
        self.assert_completed_route(result)


if __name__ == "__main__":
    unittest.main()

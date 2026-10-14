"""Frame, scan and watchdog regressions without ROS or robot connections.

Lab partners: Ding Jiang, Tongning Zhang.
"""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from team_navigate_to_goal.geometry import Pose2D, RigidTransform, StartFrame
from team_navigate_to_goal.navigation import Navigator, VelocityCommand, Waypoint, load_waypoints
from team_navigate_to_goal.scan import process_scan
from team_navigate_to_goal.watchdog import FreshnessGuard


def quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def scan(ranges=None, angle_min=-math.pi, **kwargs):
    values = [math.inf] * 360 if ranges is None else ranges
    return process_scan(values, angle_min, 2 * math.pi / len(values), 0.08, 3.5, **kwargs)


class FrameTests(unittest.TestCase):
    def test_rotated_translated_start_uses_fixed_world_origin(self):
        origin = StartFrame()
        self.assertEqual(origin.update(5.0, -3.0, quaternion(math.pi / 2)), Pose2D(0, 0, 0))
        pose = origin.update(5.0, -2.0, quaternion(math.pi))
        self.assertAlmostEqual(pose.x, 1.0)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.yaw, math.pi / 2)

    def test_relative_heading_wrap_and_non_unit_quaternion(self):
        origin = StartFrame()
        origin.update(0, 0, tuple(2 * v for v in quaternion(math.radians(179))))
        pose = origin.update(0, 0, quaternion(math.radians(-179)))
        self.assertAlmostEqual(pose.yaw, math.radians(2))

    def test_invalid_first_odometry_does_not_set_origin(self):
        origin = StartFrame()
        for q in ((0, 0, 0, 0), (0, 0, math.nan, 1), (1e300, 0, 0, 1)):
            with self.assertRaises(ValueError):
                origin.update(0, 0, q)
            self.assertIsNone(origin.origin)

    def test_supplied_waypoints_and_conservative_tolerances(self):
        path = Path(__file__).resolve().parents[1] / "config" / "wayPoints.txt"
        waypoints = load_waypoints(path)
        self.assertEqual([(w.x, w.y) for w in waypoints], [(1.5, 0), (1.5, 1.4), (0, 1.4)])
        self.assertEqual([w.tolerance for w in waypoints], [0.08, 0.12, 0.16])


class ScanTests(unittest.TestCase):
    def test_infinite_no_returns_are_clear_but_invalid_scan_is_not(self):
        empty = scan()
        self.assertTrue(empty.observed(0))
        self.assertEqual(empty.nearest, math.inf)
        for value in (math.nan, 0.0, -1.0, -math.inf, 4.0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                scan([value] * 360)

    def test_uses_metadata_and_full_laser_to_base_transform(self):
        ranges = [math.inf] * 360
        ranges[90] = 1.0  # angle_min=0: a hit on the laser's positive y axis.
        transformed = scan(ranges, angle_min=0.0,
                           transform=RigidTransform((0.1, 0.2, 0), quaternion(-math.pi / 2)))
        self.assertAlmostEqual(transformed.points[0][0], 1.1)
        self.assertAlmostEqual(transformed.points[0][1], 0.2)

    def test_circular_footprint_catches_obstacle_off_centre_ray(self):
        ranges = [math.inf] * 360
        ranges[200] = 0.50
        observation = scan(ranges)
        self.assertLess(observation.clearance(0, 0.26), 0.35)

    def test_blind_front_patch_stops_even_when_most_scan_is_good(self):
        ranges = [math.inf] * 360
        for index in range(165, 196):
            ranges[index] = math.nan
        observation = scan(ranges)
        self.assertFalse(observation.observed(0))
        controller = Navigator([Waypoint(1, 0, 0.1)])
        self.assertEqual(controller.step(Pose2D(0, 0, 0), observation, 1), VelocityCommand())

    def test_positive_below_range_min_is_conservative_emergency_stop(self):
        ranges = [math.inf] * 360
        ranges[180] = 0.04
        observation = scan(ranges)
        controller = Navigator([Waypoint(1, 0, 0.1)])
        self.assertEqual(controller.step(Pose2D(0, 0, 0), observation, 1), VelocityCommand())

    def test_rejects_partial_scan_and_vertical_laser_plane(self):
        with self.assertRaises(ValueError):
            process_scan([math.inf] * 180, 0, math.pi / 180, 0.08, 3.5)
        with self.assertRaises(ValueError):
            scan(transform=RigidTransform(rotation=(0, math.sin(math.pi / 4), 0, math.cos(math.pi / 4))))


class WatchdogTests(unittest.TestCase):
    def test_source_age_and_receipt_age_both_expire(self):
        for ros, steady in ((10.6, 20.1), (10.0, 20.6)):
            guard = FreshnessGuard()
            self.assertTrue(guard.receive(10, 10, 20))
            self.assertFalse(guard.fresh(ros, steady))

    def test_duplicate_or_reordered_messages_cannot_keep_motion_alive(self):
        guard = FreshnessGuard()
        self.assertTrue(guard.receive(10.0, 10.0, 20.0))
        for stamp in (10.0, 9.9):
            self.assertFalse(guard.receive(stamp, 10.1, 20.1))
            self.assertFalse(guard.fresh(10.1, 20.1))
        self.assertTrue(guard.receive(10.2, 10.2, 20.2))

    def test_invalid_payload_preserves_timestamp_watermark(self):
        guard = FreshnessGuard()
        guard.receive(10, 10, 20)
        guard.invalidate("malformed sensor data")
        self.assertFalse(guard.receive(10, 10.1, 20.1))
        self.assertTrue(guard.receive(10.2, 10.2, 20.2))

    def test_invalid_or_future_timestamps_cannot_start_motion(self):
        for stamp in (0, math.nan, math.inf, 10.2, 9.0):
            guard = FreshnessGuard()
            self.assertFalse(guard.receive(stamp, 10, 20))
            self.assertFalse(guard.fresh(10, 20))


if __name__ == "__main__":
    unittest.main()

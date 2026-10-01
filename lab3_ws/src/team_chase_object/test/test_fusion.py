# Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
"""Synthetic sensor geometries; no ROS installation needed."""
import math
import unittest
from team_chase_object.fusion import BearingGate, FusionConfig, RigidTransform, estimate_range


# Base x forward, y left, z up -> optical x right, y down, z forward.
OPTICAL = RigidTransform(rotation=(0.5, -0.5, 0.5, 0.5))
IDENTITY = RigidTransform()


class FusionTests(unittest.TestCase):
    def estimate(self, ranges, **kwargs):
        args = dict(angle_min=-0.1, angle_increment=0.02, range_min=0.05,
                    range_max=8.0, bearing=0.0, bearing_min=-0.14,
                    bearing_max=0.14, scan_to_camera=OPTICAL, scan_to_base=IDENTITY)
        args.update(kwargs)
        return estimate_range(ranges, **args)

    def test_optical_transform_is_correct_and_includes_translation(self):
        self.assertEqual(OPTICAL.apply((2, 0, 0)), (0, 0, 2))
        self.assertEqual(OPTICAL.apply((0, 1, 0)), (-1, 0, 0))
        self.assertEqual(OPTICAL.apply((0, 0, 1)), (0, -1, 0))
        t = RigidTransform((1, 2, 3), (0, 0, 0, 2))
        self.assertEqual(t.apply((4, 5, 6)), (5, 7, 9))

    def test_estimates_target_and_rejects_background(self):
        result = self.estimate([3]*3 + [1]*5 + [3]*3)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.distance, 1.0, delta=0.002)
        self.assertAlmostEqual(result.bearing, 0.0, places=6)
        self.assertEqual(result.points, 5)

    def test_left_target_has_positive_base_bearing(self):
        result = self.estimate([1]*11, angle_min=0.1, bearing=0.2,
                               bearing_min=0.05, bearing_max=0.35)
        self.assertGreater(result.bearing, 0)
        self.assertAlmostEqual(result.bearing, 0.2, places=6)

    def test_sensor_offset_affects_base_distance(self):
        result = self.estimate([1]*11, scan_to_base=RigidTransform((0.2, 0, 0)))
        self.assertAlmostEqual(result.distance, 1.2, delta=0.005)

    def test_camera_translation_changes_corresponding_scan_sector(self):
        # Camera is 0.2 m LEFT of lidar: target at (1, .2) is straight ahead of camera.
        camera = RigidTransform((0.2, 0, 0), OPTICAL.rotation)
        result = self.estimate([math.hypot(1, .2)]*3,
            angle_min=math.atan2(.2, 1)-0.01, angle_increment=.01,
            bearing_min=-.03, bearing_max=.03, scan_to_camera=camera)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.bearing, math.atan2(.2, 1), places=6)

    def test_invalid_ranges_and_single_speckle(self):
        self.assertIsNone(self.estimate([float('inf'), float('nan'), 0, -1, 99]))
        self.assertIsNone(self.estimate([float('inf')]*5+[1]+[float('inf')]*5))

    def test_nearest_isolated_speckle_does_not_beat_coherent_target(self):
        r = self.estimate([float('inf'), .2, float('inf'), 1, 1, 1, 1])
        self.assertGreater(r.distance, .95)

    def test_partial_and_negative_increment_scan(self):
        r = self.estimate([1]*11, angle_min=.1, angle_increment=-.02)
        self.assertAlmostEqual(r.bearing, 0, places=6)

    def test_wraparound_cluster_is_merged(self):
        ranges = [float('inf')]*360
        ranges[0] = ranges[-1] = 1
        r = self.estimate(ranges, angle_min=0, angle_increment=math.pi/180)
        self.assertIsNotNone(r)
        self.assertEqual(r.points, 2)
        self.assertAlmostEqual(r.bearing, -math.pi/360, delta=.0001)

    def test_no_scan_points_in_camera_sector(self):
        self.assertIsNone(self.estimate([1]*10, angle_min=1.0))

    def test_invalid_metadata_and_config(self):
        for kwargs in [dict(angle_increment=0), dict(range_max=0), dict(bearing=float('nan')),
                       dict(bearing_min=.2), dict(bearing_max=math.pi), dict(range_min=-1)]:
            self.assertIsNone(self.estimate([1]*11, **kwargs))
        for kwargs in [dict(roi_fraction=0), dict(cluster_gap=float('nan')), dict(min_cluster_points=0)]:
            with self.assertRaises(ValueError):
                FusionConfig(**kwargs)
        with self.assertRaises(ValueError):
            RigidTransform(rotation=(0, 0, 0, 0))

    def test_loss_blocks_queued_old_positive_bearing_until_new_detection(self):
        gate = BearingGate()
        self.assertTrue(gate.allows(9.9, 10.0))
        gate.invalidate(10.0, 10.02)
        self.assertFalse(gate.allows(9.99, 10.05))
        self.assertFalse(gate.allows(10.0, 10.05))
        self.assertTrue(gate.allows(10.03, 10.05))

    def test_loss_gate_recovers_after_clock_reset_and_bad_timestamp(self):
        gate = BearingGate()
        gate.invalidate(1000, 10.0)  # corrupt future stamp must not block forever
        self.assertFalse(gate.allows(9.99, 10.01))
        self.assertTrue(gate.allows(10.02, 10.02))
        self.assertTrue(gate.allows(1.0, 1.02))  # actual ROS time moved backwards


if __name__ == '__main__':
    unittest.main()

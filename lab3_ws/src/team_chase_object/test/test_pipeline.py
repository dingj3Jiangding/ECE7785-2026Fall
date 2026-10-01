"""Synthetic camera -> laser geometry -> controller checks (not hardware tests).

Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
"""
import math
import unittest

import cv2
import numpy as np

from team_chase_object.control import ChaseController, PolarObservation, VelocityCommand
from team_chase_object.detection import CameraModel, DetectorConfig, detect_target, detection_bearings
from team_chase_object.fusion import RigidTransform, estimate_range


class PipelineTests(unittest.TestCase):
    camera = CameraModel(np.array([[400.0, 0, 319.5], [0, 400.0, 239.5], [0, 0, 1.0]]))
    optical = RigidTransform(rotation=(0.5, -0.5, 0.5, 0.5))

    def observation(self, distance, theta, stamp):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        u = int(round(319.5 - 400 * math.tan(theta)))
        cv2.rectangle(image, (u-40, 170), (u+40, 310), (0, 255, 0), -1)
        detection = detect_target(image, DetectorConfig())
        bearing, lo, hi = detection_bearings(detection, self.camera)
        angles = [-math.pi + i*math.pi/180 for i in range(360)]
        scan = [distance if abs(a-theta) < 0.045 else 3.0 for a in angles]
        target = estimate_range(scan, -math.pi, math.pi/180, .05, 8.0,
            bearing, lo, hi, self.optical, RigidTransform())
        self.assertIsNotNone(target)
        return PolarObservation(True, target.bearing, target.distance, detection.confidence, stamp)

    def test_pixels_to_forward_left_and_reverse_right_commands(self):
        for distance, theta, v_sign, w_sign in [(1, .2, 1, 1), (.35, -.2, -1, -1)]:
            with self.subTest(distance=distance, theta=theta):
                controller = ChaseController()
                obs = self.observation(distance, theta, 10)
                self.assertTrue(controller.receive(obs, 10, 20))
                command = controller.step(10, 20)
                self.assertGreater(command.linear_x*v_sign, 0)
                self.assertGreater(command.angular_z*w_sign, 0)

    def test_ideal_robot_converges_and_stays_stopped(self):
        # Narrow, explicitly ideal scenario: instant velocity response, no wheel
        # slip, no latency and a stationary target initially about 0.96 m away.
        # This proves controller/geometry consistency, not the 5 s hardware demo.
        controller = ChaseController()
        x = y = heading = 0.0
        target_x, target_y, dt = .95, .12, .05
        stopped = 0
        for i in range(140):
            dx, dy = target_x-x, target_y-y
            theta = math.atan2(dy, dx)-heading
            now = 10+i*dt
            obs = self.observation(math.hypot(dx, dy), theta, now)
            self.assertTrue(controller.receive(obs, now, 20+i*dt))
            command = controller.step(now, 20+i*dt)
            x += command.linear_x*math.cos(heading)*dt
            y += command.linear_x*math.sin(heading)*dt
            heading += command.angular_z*dt
            stopped = stopped+1 if command == VelocityCommand() else 0
        self.assertGreaterEqual(stopped, 20)
        self.assertLessEqual(abs(math.hypot(target_x-x, target_y-y)-.6), .055)
        self.assertLessEqual(abs(math.atan2(target_y-y, target_x-x)-heading), .06)
        self.assertEqual(controller.step(18, 28), VelocityCommand())


if __name__ == '__main__':
    unittest.main()

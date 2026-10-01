# Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
"""Synthetic camera checks; these tests do not need ROS or a running robot."""

import math
import unittest

import cv2
import numpy as np

from team_chase_object.detection import (
    CameraModel, Detection, DetectorConfig, annotate_detection, detect_target,
    detection_bearings,
)


class DetectionTests(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((120, 200, 3), dtype=np.uint8)

    def test_largest_green_blob_and_confidence(self):
        cv2.rectangle(self.image, (10, 30), (39, 69), (0, 255, 0), -1)
        cv2.rectangle(self.image, (110, 20), (169, 89), (0, 255, 0), -1)
        detection = detect_target(self.image, DetectorConfig())
        self.assertIsNotNone(detection)
        self.assertEqual(detection.bbox, (110, 20, 60, 70))
        self.assertAlmostEqual(detection.centroid[0], 139.5)
        self.assertGreater(detection.confidence, 0.9)
        self.assertLessEqual(detection.confidence, 1.0)

    def test_missing_wrong_color_and_small_noise_are_rejected(self):
        self.assertIsNone(detect_target(self.image, DetectorConfig()))
        cv2.rectangle(self.image, (20, 20), (70, 70), (255, 0, 0), -1)
        cv2.rectangle(self.image, (80, 10), (85, 15), (0, 255, 0), -1)
        self.assertIsNone(detect_target(self.image, DetectorConfig()))

    def test_hue_wrap_selects_red_on_both_sides_of_zero(self):
        hsv = np.zeros_like(self.image)
        hsv[20:60, 10:50] = (175, 220, 220)
        hsv[20:80, 100:160] = (5, 220, 220)
        image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        config = DetectorConfig((170, 70, 60), (10, 255, 255))
        detection = detect_target(image, config)
        self.assertEqual(detection.bbox, (100, 20, 60, 60))
        image[:, 90:] = 0
        self.assertEqual(detect_target(image, config).bbox, (10, 20, 40, 40))

    def test_morphology_removes_isolated_noise(self):
        self.image[10, 10] = (0, 255, 0)
        cv2.rectangle(self.image, (60, 30), (89, 69), (0, 255, 0), -1)
        detection = detect_target(self.image, DetectorConfig(min_area=1))
        self.assertEqual(detection.bbox, (60, 30, 30, 40))

    def test_compressed_camera_jpeg_roundtrip_retains_target_and_bearing(self):
        cv2.rectangle(self.image, (30, 30), (89, 89), (0, 255, 0), -1)
        success, encoded = cv2.imencode(".jpg", self.image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        self.assertTrue(success)
        decoded = cv2.imdecode(np.frombuffer(encoded.tobytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertIsNotNone(decoded)
        detection = detect_target(decoded, DetectorConfig())
        self.assertIsNotNone(detection)
        self.assertAlmostEqual(detection.centroid[0], 59.5, delta=1.0)
        self.assertAlmostEqual(detection.centroid[1], 59.5, delta=1.0)
        self.assertGreater(detection.confidence, 0.8)
        bearing, minimum, maximum = detection_bearings(
            detection, CameraModel.from_fov(200, 120, 62.2),
        )
        self.assertGreater(bearing, 0.0)
        self.assertLess(minimum, bearing)
        self.assertGreater(maximum, bearing)

    def test_invalid_configuration_and_images(self):
        for kwargs in (
            {"hsv_lower": (180, 0, 0)}, {"hsv_upper": (30, 20, 255)},
            {"min_area": float("nan")}, {"min_area": 0}, {"morph_kernel": 2},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                DetectorConfig(**kwargs)
        with self.assertRaises(ValueError):
            detect_target(np.zeros((10, 10), dtype=np.uint8), DetectorConfig())
        with self.assertRaises(ValueError):
            detect_target(self.image.astype(float), DetectorConfig())

    def test_annotation_does_not_change_original(self):
        annotated = annotate_detection(self.image, None)
        self.assertTrue(np.all(self.image == 0))
        self.assertTrue(np.any(annotated != 0))


class BearingTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraModel(np.array([[100., 0., 99.5], [0., 100., 59.5], [0., 0., 1.]]))

    def test_bearing_sign_center_and_ordered_bounds(self):
        self.assertAlmostEqual(self.camera.bearing(99.5, 59.5), 0.0)
        self.assertGreater(self.camera.bearing(20, 59.5), 0)
        self.assertLess(self.camera.bearing(180, 59.5), 0)
        obj = Detection((39.5, 59.5), (20, 40, 40, 40), 1500.0, 0.9)
        bearing, minimum, maximum = detection_bearings(obj, self.camera)
        self.assertAlmostEqual(bearing, math.atan((99.5 - 39.5) / 100))
        self.assertLess(minimum, bearing)
        self.assertGreater(maximum, bearing)

    def test_calibration_scaling_preserves_rays(self):
        resized = CameraModel.from_calibration(
            400, 240, 200, 120, self.camera.matrix.reshape(-1),
        )
        self.assertAlmostEqual(resized.bearing(40, 60), self.camera.bearing(20, 30))

    def test_distortion_is_removed_before_bearing(self):
        coefficients = (0.25, -0.05, 0.002, 0.001, 0.0)
        camera = CameraModel(self.camera.matrix, coefficients, "plumb_bob")
        point = np.array([[[0.5, 0.15, 1.0]]], dtype=np.float64)
        pixels, _ = cv2.projectPoints(point, np.zeros(3), np.zeros(3), camera.matrix,
                                      np.asarray(coefficients))
        u, v = pixels[0, 0]
        self.assertAlmostEqual(camera.bearing(float(u), float(v)), -math.atan(0.5), places=5)

    def test_fisheye_distortion_is_removed(self):
        coefficients = (0.1, -0.02, 0.0, 0.0)
        camera = CameraModel(self.camera.matrix, coefficients, "equidistant")
        point = np.array([[[0.4, 0.1, 1.0]]], dtype=np.float64)
        pixels, _ = cv2.fisheye.projectPoints(point, np.zeros(3), np.zeros(3),
                                             camera.matrix, np.asarray(coefficients))
        u, v = pixels[0, 0]
        self.assertAlmostEqual(camera.bearing(float(u), float(v)), -math.atan(0.4), places=6)

    def test_fov_fallback_and_invalid_calibration(self):
        fallback = CameraModel.from_fov(200, 120, 60.0)
        self.assertAlmostEqual(fallback.bearing(99.5, 59.5), 0.0)
        self.assertAlmostEqual(fallback.bearing(-0.5, 59.5), math.radians(30))
        with self.assertRaises(ValueError):
            CameraModel.from_fov(200, 120, 180)
        with self.assertRaises(ValueError):
            CameraModel(np.zeros((3, 3)))
        with self.assertRaises(ValueError):
            CameraModel(self.camera.matrix, (0.0, 0.0), "unknown")


if __name__ == "__main__":
    unittest.main()

"""ROS-independent colour segmentation and camera bearing calculations.

Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
Bearings are radians, positive to the LEFT of the optical +Z axis.
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectorConfig:
    hsv_lower: Tuple[int, int, int] = (35, 70, 60)
    hsv_upper: Tuple[int, int, int] = (85, 255, 255)
    min_area: float = 150.0
    morph_kernel: int = 3

    def __post_init__(self):
        for name, bound in (("hsv_lower", self.hsv_lower), ("hsv_upper", self.hsv_upper)):
            if len(bound) != 3 or any(
                isinstance(v, bool) or not isinstance(v, (int, np.integer))
                for v in bound
            ):
                raise ValueError(f"{name} must contain three integer HSV values")
            if not (0 <= bound[0] <= 179 and all(0 <= v <= 255 for v in bound[1:])):
                raise ValueError(f"{name}: hue must be 0..179, saturation/value 0..255")
        if any(self.hsv_lower[i] > self.hsv_upper[i] for i in (1, 2)):
            raise ValueError("lower saturation/value must not exceed upper bounds")
        if not math.isfinite(self.min_area) or self.min_area <= 0:
            raise ValueError("min_area must be finite and positive")
        if isinstance(self.morph_kernel, bool) or not isinstance(self.morph_kernel, int):
            raise ValueError("morph_kernel must be an odd positive integer")
        if self.morph_kernel < 1 or self.morph_kernel % 2 == 0:
            raise ValueError("morph_kernel must be an odd positive integer (1 disables it)")


@dataclass(frozen=True)
class Detection:
    centroid: Tuple[float, float]
    bbox: Tuple[int, int, int, int]
    area: float
    confidence: float


@dataclass(frozen=True)
class CameraModel:
    """Intrinsics for the actual image resolution, with optional raw distortion."""

    matrix: np.ndarray
    distortion: Tuple[float, ...] = ()
    distortion_model: str = ""

    def __post_init__(self):
        matrix = np.asarray(self.matrix, dtype=np.float64)
        if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
            raise ValueError("camera matrix must be a finite 3x3 matrix")
        if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
            raise ValueError("camera focal lengths must be positive")
        if not np.allclose(matrix[2], [0.0, 0.0, 1.0]):
            raise ValueError("camera matrix must have final row [0, 0, 1]")
        distortion = tuple(float(v) for v in self.distortion)
        if not all(math.isfinite(v) for v in distortion):
            raise ValueError("distortion coefficients must be finite")
        if distortion:
            if self.distortion_model in ("plumb_bob", "rational_polynomial"):
                if len(distortion) not in (4, 5, 8, 12, 14):
                    raise ValueError("unsupported pinhole distortion coefficient count")
            elif self.distortion_model == "equidistant":
                if len(distortion) != 4:
                    raise ValueError("equidistant distortion requires four coefficients")
            else:
                raise ValueError(f"unsupported distortion model: {self.distortion_model!r}")
        object.__setattr__(self, "matrix", matrix.copy())
        object.__setattr__(self, "distortion", distortion)

    @classmethod
    def from_fov(cls, width: int, height: int, horizontal_fov_deg: float):
        if width < 1 or height < 1:
            raise ValueError("image dimensions must be positive")
        if not math.isfinite(horizontal_fov_deg) or not 0 < horizontal_fov_deg < 180:
            raise ValueError("horizontal_fov_deg must be between 0 and 180")
        fx = width / (2.0 * math.tan(math.radians(horizontal_fov_deg) / 2.0))
        return cls(np.array([
            [fx, 0.0, (width - 1) / 2.0],
            [0.0, fx, (height - 1) / 2.0],
            [0.0, 0.0, 1.0],
        ]))

    @classmethod
    def from_calibration(
        cls, width: int, height: int, calibration_width: int,
        calibration_height: int, k: Sequence[float], d: Sequence[float] = (),
        distortion_model: str = "",
    ):
        """Scale calibration to a resized image (no arbitrary crop is inferred)."""
        if min(width, height, calibration_width, calibration_height) < 1:
            raise ValueError("image and calibration dimensions must be positive")
        if len(k) != 9:
            raise ValueError("CameraInfo K must contain nine values")
        matrix = np.asarray(k, dtype=np.float64).reshape(3, 3).copy()
        matrix[0, :] *= width / float(calibration_width)
        matrix[1, :] *= height / float(calibration_height)
        return cls(matrix, tuple(d), distortion_model)

    def bearing(self, u: float, v: float) -> float:
        if not (math.isfinite(u) and math.isfinite(v)):
            raise ValueError("pixel coordinates must be finite")
        if self.distortion:
            points = np.array([[[u, v]]], dtype=np.float64)
            coefficients = np.asarray(self.distortion, dtype=np.float64)
            if self.distortion_model == "equidistant":
                normalized = cv2.fisheye.undistortPoints(points, self.matrix, coefficients)
            else:
                normalized = cv2.undistortPoints(points, self.matrix, coefficients)
            x = float(normalized[0, 0, 0])
        else:
            normalized = np.linalg.solve(self.matrix, np.array([u, v, 1.0]))
            x = float(normalized[0] / normalized[2])
        if not math.isfinite(x):
            raise ValueError("camera projection produced a non-finite bearing")
        return math.atan2(-x, 1.0)


def detect_target(image: np.ndarray, config: DetectorConfig) -> Optional[Detection]:
    """Return the largest eligible HSV blob in an uint8 BGR image, or None."""
    if (
        not isinstance(image, np.ndarray) or image.dtype != np.uint8
        or image.ndim != 3 or image.shape[2] != 3
        or image.shape[0] < 1 or image.shape[1] < 1
    ):
        raise ValueError("expected a non-empty uint8 BGR image")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower, upper = config.hsv_lower, config.hsv_upper
    if lower[0] <= upper[0]:
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    else:
        # OpenCV hue wraps at 180: e.g. [170, S, V] through [10, S, V] selects red.
        high_mask = cv2.inRange(hsv, np.array(lower), np.array((179, upper[1], upper[2])))
        low_mask = cv2.inRange(hsv, np.array((0, lower[1], lower[2])), np.array(upper))
        mask = cv2.bitwise_or(high_mask, low_mask)
    if config.morph_kernel > 1:
        kernel = np.ones((config.morph_kernel, config.morph_kernel), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    eligible = [c for c in contours if cv2.contourArea(c) >= config.min_area]
    if not eligible:
        return None
    contour = max(eligible, key=cv2.contourArea)
    moments = cv2.moments(contour)
    if moments["m00"] <= 0:
        return None
    centroid = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
    bbox = tuple(int(v) for v in cv2.boundingRect(contour))
    area = float(cv2.contourArea(contour))
    # This is a geometric fill score, not a probability or image-area fraction.
    confidence = min(1.0, max(0.0, area / float(bbox[2] * bbox[3])))
    return Detection(centroid, bbox, area, confidence)


def detection_bearings(detection: Detection, camera: CameraModel) -> Tuple[float, float, float]:
    """Return centroid bearing and ordered bounds for the horizontal bounding box."""
    u, v = detection.centroid
    x, _, width, _ = detection.bbox
    bearing = camera.bearing(u, v)
    edges = [camera.bearing(float(x), v), camera.bearing(float(x + width - 1), v)]
    return bearing, min(edges + [bearing]), max(edges + [bearing])


def annotate_detection(image: np.ndarray, detection: Optional[Detection]) -> np.ndarray:
    result = image.copy()
    if detection is None:
        cv2.putText(result, "No target", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return result
    x, y, width, height = detection.bbox
    cv2.rectangle(result, (x, y), (x + width - 1, y + height - 1), (0, 255, 255), 2)
    point = tuple(int(round(v)) for v in detection.centroid)
    cv2.circle(result, point, 4, (255, 0, 255), -1)
    cv2.putText(result, f"Target fill={detection.confidence:.2f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return result

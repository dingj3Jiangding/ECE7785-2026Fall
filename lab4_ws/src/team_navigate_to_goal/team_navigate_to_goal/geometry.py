"""Planar pose helpers, including the instructor's start-frame correction.

Lab partners: Ding Jiang, Tongning Zhang.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def normalized_quaternion(quaternion):
    if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
        raise ValueError("Quaternion must contain four finite values")
    norm = math.sqrt(sum(v * v for v in quaternion))
    if not math.isfinite(norm) or norm < 1e-10:
        raise ValueError("Quaternion must not be zero or overflow")
    return tuple(v / norm for v in quaternion)


def quaternion_yaw(quaternion) -> float:
    x, y, z, w = normalized_quaternion(quaternion)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x, self.y, self.yaw)):
            raise ValueError("Pose must be finite")


class StartFrame:
    """R(-yaw0) * (position-position0); first valid pose defines the origin."""

    def __init__(self):
        self.origin: Optional[Pose2D] = None

    def update(self, x, y, quaternion) -> Pose2D:
        current = Pose2D(x, y, quaternion_yaw(quaternion))
        if self.origin is None:
            self.origin = current
        dx, dy = x - self.origin.x, y - self.origin.y
        c, s = math.cos(self.origin.yaw), math.sin(self.origin.yaw)
        return Pose2D(c * dx + s * dy, -s * dx + c * dy,
                      wrap_angle(current.yaw - self.origin.yaw))


@dataclass(frozen=True)
class RigidTransform:
    """Source-to-target transform, using the same convention as Lab 3 fusion."""

    translation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self):
        if len(self.translation) != 3 or not all(math.isfinite(v) for v in self.translation):
            raise ValueError("Translation must contain three finite values")
        object.__setattr__(self, "rotation", normalized_quaternion(self.rotation))

    def apply(self, point):
        x, y, z, w = self.rotation
        px, py, pz = point
        a, b, c = y * pz - z * py, z * px - x * pz, x * py - y * px
        return (px + 2 * (w * a + y * c - z * b) + self.translation[0],
                py + 2 * (w * b + z * a - x * c) + self.translation[1],
                pz + 2 * (w * c + x * b - y * a) + self.translation[2])

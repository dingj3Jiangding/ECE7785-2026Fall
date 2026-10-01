"""Geometry and conservative camera/LaserScan association, independent of ROS.

Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
"""
from dataclasses import dataclass
import math
from statistics import median
from typing import Optional, Sequence, Tuple

Point = Tuple[float, float, float]


@dataclass(frozen=True)
class RigidTransform:
    """Source-to-target translation and quaternion (x, y, z, w)."""
    translation: Point = (0.0, 0.0, 0.0)
    rotation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self):
        if len(self.translation) != 3 or len(self.rotation) != 4:
            raise ValueError('Invalid transform dimensions')
        if not all(math.isfinite(v) for v in (*self.translation, *self.rotation)):
            raise ValueError('Transform must be finite')
        n = math.sqrt(sum(v*v for v in self.rotation))
        if n < 1e-10:
            raise ValueError('Zero quaternion')
        object.__setattr__(self, 'rotation', tuple(v/n for v in self.rotation))

    def apply(self, p: Point) -> Point:
        x, y, z, w = self.rotation
        px, py, pz = p
        # Quaternion-vector rotation: v + 2w(q cross v) + 2(q cross(q cross v)).
        a, b, c = y*pz-z*py, z*px-x*pz, x*py-y*px
        return (px+2*(w*a+y*c-z*b)+self.translation[0],
                py+2*(w*b+z*a-x*c)+self.translation[1],
                pz+2*(w*c+x*b-y*a)+self.translation[2])


@dataclass(frozen=True)
class FusionConfig:
    roi_fraction: float = 0.7
    cluster_gap: float = 0.15
    min_cluster_points: int = 2

    def __post_init__(self):
        if not math.isfinite(self.roi_fraction) or not 0 < self.roi_fraction <= 1:
            raise ValueError('roi_fraction must be in (0, 1]')
        if not math.isfinite(self.cluster_gap) or self.cluster_gap <= 0:
            raise ValueError('cluster_gap must be positive')
        if type(self.min_cluster_points) is not int or self.min_cluster_points < 1:
            raise ValueError('min_cluster_points must be a positive integer')


@dataclass(frozen=True)
class RangeEstimate:
    bearing: float
    distance: float
    points: int


class BearingGate:
    """Prevent queued pre-loss camera observations from reviving a target."""

    def __init__(self):
        self.lost_stamp = -math.inf
        self.last_now = None

    def _clock(self, now):
        if self.last_now is not None and now < self.last_now - 0.1:
            # A real ROS clock reset starts a new observation epoch.
            self.lost_stamp = -math.inf
        self.last_now = now

    def invalidate(self, stamp, now):
        self._clock(now)
        # An unusable timestamp cannot poison the watermark arbitrarily far ahead.
        cutoff = stamp if math.isfinite(stamp) and 0 < stamp <= now + 0.1 else now
        self.lost_stamp = max(self.lost_stamp, cutoff)

    def allows(self, stamp, now):
        self._clock(now)
        return math.isfinite(stamp) and stamp > self.lost_stamp


def estimate_range(ranges: Sequence[float], angle_min: float, angle_increment: float,
                   range_min: float, range_max: float, bearing: float,
                   bearing_min: float, bearing_max: float,
                   scan_to_camera: RigidTransform, scan_to_base: RigidTransform,
                   config: FusionConfig = FusionConfig()) -> Optional[RangeEstimate]:
    """Choose the nearest coherent cluster in the central camera bearing sector.

    Project laser points into the optical frame, so sensor translations as well
    as rotations are respected. This requires the target to intersect the laser
    plane. It cannot distinguish a foreground occluder from the target.
    """
    values = (angle_min, angle_increment, range_min, range_max,
              bearing, bearing_min, bearing_max)
    if len(ranges) == 0 or not all(math.isfinite(v) for v in values):
        return None
    if (angle_increment == 0 or range_min < 0 or range_max <= range_min
            or not -math.pi/2 < bearing_min <= bearing <= bearing_max < math.pi/2
            or bearing_max <= bearing_min):
        return None
    lo = bearing + (bearing_min-bearing)*config.roi_fraction
    hi = bearing + (bearing_max-bearing)*config.roi_fraction
    clusters = []
    current = []
    for i, r in enumerate(ranges):
        point = None
        if math.isfinite(r) and r > 0 and range_min <= r <= range_max:
            theta = angle_min + i*angle_increment
            candidate = (r*math.cos(theta), r*math.sin(theta), 0.0)
            cam = scan_to_camera.apply(candidate)
            if cam[2] > 0 and lo <= math.atan2(-cam[0], cam[2]) <= hi:
                point = candidate
        if point is None:
            if current:
                clusters.append(current)
                current = []
            continue
        if current and math.dist(point, current[-1][1]) > config.cluster_gap:
            clusters.append(current)
            current = []
        current.append((i, point))
    if current:
        clusters.append(current)
    # A 360-degree scan can split a target at its first/last sample boundary.
    covers_circle = abs(abs(angle_increment)*len(ranges)-2*math.pi) <= 2*abs(angle_increment)
    if (covers_circle and len(clusters) > 1 and clusters[0][0][0] == 0
            and clusters[-1][-1][0] == len(ranges)-1
            and math.dist(clusters[0][0][1], clusters[-1][-1][1]) <= config.cluster_gap):
        clusters[0] = clusters[-1] + clusters[0]
        clusters.pop()
    eligible = [c for c in clusters if len(c) >= config.min_cluster_points]
    if not eligible:
        return None
    # Median avoids isolated range noise; coherent-cluster requirement rejects single speckles.
    cluster = min(eligible, key=lambda c: median(math.hypot(p[0], p[1]) for _, p in c))
    base_points = [scan_to_base.apply(p) for _, p in cluster]
    x, y = median(p[0] for p in base_points), median(p[1] for p in base_points)
    distance = math.hypot(x, y)
    if not math.isfinite(distance) or distance <= 0:
        return None
    return RangeEstimate(math.atan2(y, x), distance, len(cluster))

"""Transform a 360-degree LaserScan into local collision and wall observations.

Lab partners: Ding Jiang, Tongning Zhang.
"""

from dataclasses import dataclass
import math
from typing import Tuple

from .geometry import RigidTransform, wrap_angle


@dataclass(frozen=True)
class ScanConfig:
    min_valid_fraction: float = 0.5

    def __post_init__(self):
        if not math.isfinite(self.min_valid_fraction) or not 0 < self.min_valid_fraction <= 1:
            raise ValueError("min_valid_fraction must be in (0, 1]")


@dataclass(frozen=True)
class ScanSnapshot:
    points: Tuple[Tuple[float, float], ...]
    rays: Tuple[Tuple[float, bool], ...]
    range_max: float
    min_valid_fraction: float
    resolution: float

    @property
    def nearest(self):
        return min((math.hypot(x, y) for x, y in self.points), default=math.inf)

    def observed(self, direction, half_width=math.radians(25)):
        """Unknown sectors cannot be used as a free route, even on a valid scan."""
        samples = sorted((wrap_angle(a - direction), valid) for a, valid in self.rays
                         if abs(wrap_angle(a - direction)) <= half_width)
        if not samples or sum(v for _, v in samples) / len(samples) < self.min_valid_fraction:
            return False
        # Also reject a contiguous blind patch, which a global fraction misses.
        valid_angles = [-half_width] + [a for a, valid in samples if valid] + [half_width]
        return max(b - a for a, b in zip(valid_angles, valid_angles[1:])) <= max(
            math.radians(20), 2.5 * self.resolution)

    def clearance(self, direction, radius, limit=math.inf):
        """Forward travel before a circle of radius touches any measured point."""
        c, s = math.cos(direction), math.sin(direction)
        distance = limit
        for x, y in self.points:
            along, across = c * x + s * y, -s * x + c * y
            if along >= 0 and abs(across) <= radius:
                distance = min(distance, max(0.0, along - math.sqrt(max(0.0, radius**2 - across**2))))
        return distance

    def sector_distance(self, direction, half_width):
        return min((math.hypot(x, y) for x, y in self.points
                    if abs(wrap_angle(math.atan2(y, x) - direction)) <= half_width), default=math.inf)

    def wall(self, side, max_distance):
        """Fit a local tangent around the nearest side return; +1 is left.

        The local fit handles a rotated box face. No fixed obstacle orientation
        or fixed beam index is assumed. Very short clusters use distance only.
        """
        candidates = [(x, y) for x, y in self.points
                      if math.radians(20) <= side * math.atan2(y, x) <= math.radians(140)
                      and math.hypot(x, y) <= max_distance]
        if not candidates:
            return None
        closest = min(candidates, key=lambda p: math.hypot(*p))
        local = [(x, y) for x, y in candidates
                 if math.hypot(x - closest[0], y - closest[1]) <= 0.25]
        distance = math.hypot(*closest)
        if len(local) < 3:
            return distance, 0.0
        mx, my = sum(p[0] for p in local) / len(local), sum(p[1] for p in local) / len(local)
        xx = sum((x - mx)**2 for x, y in local)
        yy = sum((y - my)**2 for x, y in local)
        xy = sum((x - mx) * (y - my) for x, y in local)
        tangent = 0.5 * math.atan2(2 * xy, xx - yy)
        # A perpendicular face has two equally good tangent directions. Turn
        # away from the obstacle, preserving the chosen bypass side.
        if abs(abs(tangent) - math.pi / 2) < 1e-6:
            tangent = -side * math.pi / 2
        return distance, tangent


def process_scan(ranges, angle_min, angle_increment, range_min, range_max,
                 transform=RigidTransform(), config=ScanConfig()):
    if (len(ranges) < 12 or not all(math.isfinite(v) for v in
            (angle_min, angle_increment, range_min, range_max))
            or angle_increment == 0 or range_min < 0 or range_max <= range_min):
        raise ValueError("Invalid LaserScan metadata")
    coverage = abs(angle_increment) * len(ranges)
    if abs(coverage - 2 * math.pi) > max(3 * abs(angle_increment), math.radians(10)):
        raise ValueError("This controller requires a 360-degree LaserScan")
    points, rays = [], []
    for i, value in enumerate(ranges):
        angle = angle_min + i * angle_increment
        c, s = math.cos(angle), math.sin(angle)
        # Ray orientation is transformed without translation; the finite hit
        # position below includes the measured laser-to-base translation.
        direction = transform.apply((c, s, 0.0))
        direction = (direction[0] - transform.translation[0],
                     direction[1] - transform.translation[1])
        if math.hypot(*direction) < 0.9:
            raise ValueError("Laser scan plane is not approximately horizontal")
        valid = value == math.inf or (math.isfinite(value) and 0 < value <= range_max)
        # A positive reading below range_min is kept conservatively as a close
        # return. Zero, NaN, negative and beyond-range readings stay unknown.
        rays.append((math.atan2(direction[1], direction[0]), valid))
        if valid and math.isfinite(value):
            x, y, _ = transform.apply((value * c, value * s, 0.0))
            points.append((x, y))
    if sum(valid for _, valid in rays) / len(rays) < config.min_valid_fraction:
        raise ValueError("Too few valid laser rays")
    return ScanSnapshot(tuple(points), tuple(rays), range_max,
                        config.min_valid_fraction, abs(angle_increment))

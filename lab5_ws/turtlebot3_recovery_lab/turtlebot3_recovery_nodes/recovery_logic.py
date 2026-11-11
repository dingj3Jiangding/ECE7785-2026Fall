"""ROS-independent endpoint checks; Nav2 remains responsible for path feasibility."""

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    cells: Sequence[int]

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0 or len(self.cells) != self.width * self.height:
            raise ValueError('Occupancy grid dimensions/data are invalid')
        if not all(math.isfinite(v) for v in (
                self.resolution, self.origin_x, self.origin_y, self.origin_yaw)):
            raise ValueError('Occupancy grid geometry must be finite')
        if self.resolution <= 0.0:
            raise ValueError('Occupancy grid resolution must be positive')


def yaw_from_quaternion(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Quaternion must be finite')
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-9:
        raise ValueError('Quaternion has zero length')
    x, y, z, w = (v / norm for v in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def grid_from_message(message):
    """Accept the OccupancyGrid interface without importing nav_msgs here."""
    info = message.info
    return Grid(info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y,
                yaw_from_quaternion(info.origin.orientation), message.data)


def footprint_is_clear(grid, x, y, radius, occupied_threshold):
    """Check every cell intersecting a circular robot footprint, including borders.

    Unknown cells and any footprint extending outside the map are rejected.
    Rotated map origins are supported; inflation costs need a different threshold
    from the occupancy probability in the static map.
    """
    if not all(math.isfinite(v) for v in (x, y, radius)) or radius < 0.0:
        return False
    dx, dy = x - grid.origin_x, y - grid.origin_y
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    gx, gy = c * dx + s * dy, -s * dx + c * dy
    resolution = grid.resolution
    if (gx - radius < 0.0 or gy - radius < 0.0
            or gx + radius >= grid.width * resolution
            or gy + radius >= grid.height * resolution):
        return False
    # Include the cell on the lower side of an exactly tangent grid boundary.
    # The circle/cell test below removes the extra cells when they do not touch.
    col_min = max(0, math.floor((gx - radius) / resolution) - 1)
    row_min = max(0, math.floor((gy - radius) / resolution) - 1)
    col_max = math.floor((gx + radius) / resolution)
    row_max = math.floor((gy + radius) / resolution)
    for row in range(row_min, row_max + 1):
        for col in range(col_min, col_max + 1):
            nearest_x = min(max(gx, col * resolution), (col + 1) * resolution)
            nearest_y = min(max(gy, row * resolution), (row + 1) * resolution)
            if math.hypot(gx - nearest_x, gy - nearest_y) > radius:
                continue
            value = grid.cells[row * grid.width + col]
            if value < 0 or value >= occupied_threshold:
                return False
    return True


def choose_recovery_waypoint(candidates, robot_x, robot_y, static_grid, costmap,
                             min_distance=0.4, clearance=0.2,
                             occupied_threshold=50, costmap_occupied_threshold=100):
    """Return nearest configured endpoint clear in both maps, or None.

    Costmap OccupancyGrid 100 is lethal; lower values include inflated costs.
    Applying a footprint radius to all inflated cells would count clearance twice.
    This checks endpoints, not a route through the maze. ComputePathToPose in the
    behavior tree must succeed before FollowPath moves toward this candidate.
    """
    if not all(math.isfinite(v) for v in (robot_x, robot_y, min_distance, clearance)):
        return None
    ranked = sorted(candidates, key=lambda p: math.hypot(p.x - robot_x, p.y - robot_y))
    for point in ranked:
        if math.hypot(point.x - robot_x, point.y - robot_y) < min_distance:
            continue
        if not footprint_is_clear(static_grid, point.x, point.y, clearance, occupied_threshold):
            continue
        if not footprint_is_clear(costmap, point.x, point.y, clearance, costmap_occupied_threshold):
            continue
        return point
    return None

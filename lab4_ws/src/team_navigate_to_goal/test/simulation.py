"""Deterministic, ROS-free navigation checks; these do not model real hardware.

Lab partners: Ding Jiang, Tongning Zhang.

The simulator integrates a differential-drive robot, casts 360 ideal lidar rays,
and checks a circular robot footprint against oriented rectangular obstacles.
It deliberately has no wheel slip, odometry drift, latency, or sensor noise.
"""

from dataclasses import dataclass, field
import argparse
import json
import math
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from team_navigate_to_goal.geometry import Pose2D  # noqa: E402
from team_navigate_to_goal.navigation import (  # noqa: E402
    NavigationConfig,
    Navigator,
    Waypoint,
)
from team_navigate_to_goal.scan import process_scan  # noqa: E402


@dataclass(frozen=True)
class Rectangle:
    """A box specified by its center, half extents, and world-frame yaw."""

    x: float
    y: float
    half_width: float = 0.20
    half_height: float = 0.16
    yaw: float = 0.0

    def local_coordinates(self, x, y):
        cosine, sine = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = x - self.x, y - self.y
        return cosine * dx + sine * dy, -sine * dx + cosine * dy

    def ray_distance(self, x, y, angle, maximum):
        """Intersect a ray with this oriented rectangle using slab intervals."""
        origin = self.local_coordinates(x, y)
        direction = (math.cos(angle - self.yaw), math.sin(angle - self.yaw))
        entry, exit_distance = 0.0, maximum
        for coordinate, component, half_extent in zip(
            origin, direction, (self.half_width, self.half_height)
        ):
            if abs(component) < 1e-12:
                if abs(coordinate) > half_extent:
                    return math.inf
                continue
            first = (-half_extent - coordinate) / component
            second = (half_extent - coordinate) / component
            entry = max(entry, min(first, second))
            exit_distance = min(exit_distance, max(first, second))
            if entry > exit_distance:
                return math.inf
        return entry if exit_distance >= entry and entry < maximum else math.inf

    def clearance(self, x, y, radius):
        """Signed circle-to-box distance; negative means footprint collision."""
        local_x, local_y = self.local_coordinates(x, y)
        outside_x = max(abs(local_x) - self.half_width, 0.0)
        outside_y = max(abs(local_y) - self.half_height, 0.0)
        return math.hypot(outside_x, outside_y) - radius


@dataclass
class SimulationResult:
    done: bool
    elapsed: float
    collision: bool
    state: str
    goal_index: int
    minimum_clearance: float
    hold_durations: list = field(default_factory=list)
    arrival_errors: list = field(default_factory=list)
    inserted_obstacle: bool = False
    trace: list = field(default_factory=list)

    def summary(self):
        return {
            "done": self.done,
            "elapsed_seconds": round(self.elapsed, 3),
            "collision": self.collision,
            "state": self.state,
            "goal_index": self.goal_index,
            "minimum_clearance_m": (
                round(self.minimum_clearance, 4)
                if math.isfinite(self.minimum_clearance)
                else None
            ),
            "hold_seconds": [round(value, 3) for value in self.hold_durations],
            "arrival_errors_m": [round(value, 4) for value in self.arrival_errors],
            "inserted_obstacle": self.inserted_obstacle,
        }


def ideal_scan(pose, obstacles=(), beam_count=360, maximum=3.5):
    increment = 2.0 * math.pi / beam_count
    ranges = []
    for index in range(beam_count):
        angle = pose.yaw - math.pi + index * increment
        ranges.append(
            min(
                (box.ray_distance(pose.x, pose.y, angle, maximum) for box in obstacles),
                default=math.inf,
            )
        )
    return process_scan(ranges, -math.pi, increment, 0.08, maximum)


def integrate(pose, command, dt):
    """Exact constant-velocity unicycle integration over one control interval."""
    velocity, angular_velocity = command.linear_x, command.angular_z
    next_yaw = pose.yaw + angular_velocity * dt
    if abs(angular_velocity) < 1e-12:
        next_x = pose.x + velocity * math.cos(pose.yaw) * dt
        next_y = pose.y + velocity * math.sin(pose.yaw) * dt
    else:
        radius = velocity / angular_velocity
        next_x = pose.x + radius * (math.sin(next_yaw) - math.sin(pose.yaw))
        next_y = pose.y - radius * (math.cos(next_yaw) - math.cos(pose.yaw))
    return Pose2D(next_x, next_y, math.atan2(math.sin(next_yaw), math.cos(next_yaw)))


def run_scenario(with_obstacles=False, obstacle_angle_degrees=0.0, limit=150.0, dt=0.05,
                 tolerances=(0.08, 0.12, 0.16)):
    """Run all three waypoints; insert a second box after the second goal hold."""
    config = NavigationConfig()
    waypoints = [Waypoint(x, y, tolerance) for (x, y), tolerance in zip(
        ((1.5, 0.0), (1.5, 1.4), (0.0, 1.4)), tolerances)]
    navigator = Navigator(waypoints, config=config)
    pose = Pose2D(0.0, 0.0, 0.0)
    obstacle_yaw = math.radians(obstacle_angle_degrees)
    obstacles = [Rectangle(1.5, 0.7, yaw=obstacle_yaw)] if with_obstacles else []
    minimum_clearance = math.inf
    hold_start = None
    hold_durations, arrival_errors, trace = [], [], []
    inserted_obstacle = False
    collision = False
    now = 0.0

    for step_index in range(round(limit / dt) + 1):
        now = step_index * dt
        if with_obstacles and navigator.goal_index >= 2 and not inserted_obstacle:
            obstacles.append(Rectangle(0.7, 1.4, yaw=-obstacle_yaw))
            inserted_obstacle = True
        command = navigator.step(pose, ideal_scan(pose, obstacles), now)

        if navigator.state == "HOLD" and hold_start is None:
            hold_start = now
            goal = waypoints[navigator.goal_index]
            arrival_errors.append(math.hypot(pose.x - goal.x, pose.y - goal.y))
        elif navigator.state != "HOLD" and hold_start is not None:
            hold_durations.append(now - hold_start)
            hold_start = None

        if step_index % 10 == 0 or navigator.state in ("DONE", "BLOCKED"):
            trace.append((now, pose.x, pose.y, pose.yaw, navigator.state, navigator.goal_index))
        if navigator.state in ("DONE", "BLOCKED"):
            break

        # Smaller collision intervals also catch a footprint crossing a box
        # between control updates; scan and control still execute at 20 Hz.
        for _ in range(5):
            pose = integrate(pose, command, dt / 5.0)
            clearance = min(
                (box.clearance(pose.x, pose.y, config.robot_radius) for box in obstacles),
                default=math.inf,
            )
            minimum_clearance = min(minimum_clearance, clearance)
            if clearance <= 0.0:
                collision = True
                break
        if collision:
            break

    return SimulationResult(
        done=navigator.state == "DONE",
        elapsed=now,
        collision=collision,
        state=navigator.state,
        goal_index=navigator.goal_index,
        minimum_clearance=minimum_clearance,
        hold_durations=hold_durations,
        arrival_errors=arrival_errors,
        inserted_obstacle=inserted_obstacle,
        trace=trace,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--angles", type=float, nargs="+", default=[0.0, 30.0, 60.0, 90.0])
    arguments = parser.parse_args()
    results = {"empty": run_scenario().summary()}
    for angle in arguments.angles:
        results[f"boxes_{angle:g}_degrees"] = run_scenario(True, angle).summary()
    print(json.dumps(results, indent=2))
    return 0 if all(result["done"] and not result["collision"] for result in results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())

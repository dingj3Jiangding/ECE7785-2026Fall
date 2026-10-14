"""Waypoint sequencing and a reactive goal / wall-follow controller.

Lab partners: Ding Jiang, Tongning Zhang.

This first version targets isolated box obstacles, not arbitrary mazes. It
keeps the bypass side until it has made progress and can see a clear corridor
to the goal. The ROS wrapper owns timestamps, TF, and all sensor watchdogs.
"""

from dataclasses import dataclass
import math
from pathlib import Path

from .geometry import wrap_angle


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    tolerance: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x, self.y, self.tolerance)) or self.tolerance <= 0:
            raise ValueError("Waypoint coordinates must be finite and tolerance positive")


def load_waypoints(path, tolerances=(0.08, 0.12, 0.16)):
    coordinates = []
    for line_number, line in enumerate(Path(path).read_text().splitlines(), 1):
        fields = line.split("#", 1)[0].split()
        if not fields:
            continue
        if len(fields) != 2:
            raise ValueError(f"{path}:{line_number}: expected x y")
        coordinates.append(tuple(float(v) for v in fields))
    if not coordinates or len(coordinates) != len(tolerances):
        raise ValueError("Supply one positive tolerance for each waypoint")
    return tuple(Waypoint(x, y, tolerance) for (x, y), tolerance in zip(coordinates, tolerances))


@dataclass(frozen=True)
class NavigationConfig:
    max_forward_speed: float = 0.15
    max_angular_speed: float = 0.8
    linear_kp: float = 0.5
    angular_kp: float = 1.5
    heading_gate: float = 0.6
    goal_hold_seconds: float = 10.0
    robot_radius: float = 0.22
    safety_margin: float = 0.04
    obstacle_trigger_distance: float = 0.65
    wall_distance: float = 0.40
    wall_follow_speed: float = 0.10
    wall_kp: float = 2.0
    wall_heading_kp: float = 1.5
    leave_progress: float = 0.10
    min_wall_follow_seconds: float = 1.0
    avoidance_timeout: float = 60.0
    preferred_turn: str = "auto"

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if name == "preferred_turn":
                continue
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0 < self.heading_gate < math.pi / 2:
            raise ValueError("heading_gate must be between 0 and pi/2")
        if self.wall_distance <= self.robot_radius + self.safety_margin:
            raise ValueError("wall_distance must exceed the inflated robot radius")
        if self.obstacle_trigger_distance <= self.safety_margin:
            raise ValueError("obstacle_trigger_distance must exceed safety_margin")
        if self.wall_follow_speed > self.max_forward_speed:
            raise ValueError("wall_follow_speed must not exceed max_forward_speed")
        if self.avoidance_timeout <= self.min_wall_follow_seconds:
            raise ValueError("avoidance_timeout must exceed min_wall_follow_seconds")
        if self.preferred_turn not in ("auto", "left", "right"):
            raise ValueError("preferred_turn must be auto, left, or right")


class Navigator:
    def __init__(self, waypoints, config=NavigationConfig()):
        self.waypoints = tuple(waypoints)
        if not self.waypoints or not all(isinstance(w, Waypoint) for w in self.waypoints):
            raise ValueError("At least one Waypoint is required")
        self.config = config
        self.goal_index = 0
        self.state = "GO_TO_GOAL"
        self.status = "waiting for sensors"
        self.hold_since = None
        self.follow_since = None
        self.hit_distance = None
        self.turn_sign = 1
        self.turning_away = False
        self.last_time = None

    def pause(self, reason):
        """A sensor interruption invalidates a continuous 10-second goal hold."""
        self.hold_since = None
        self.status = f"STOP: {reason}"

    def _turn_choice(self, scan, heading):
        if self.config.preferred_turn != "auto":
            return 1 if self.config.preferred_turn == "left" else -1
        radius = self.config.robot_radius + self.config.safety_margin
        left = scan.clearance(math.pi / 3, radius, 1.5) if scan.observed(math.pi / 3) else 0.0
        right = scan.clearance(-math.pi / 3, radius, 1.5) if scan.observed(-math.pi / 3) else 0.0
        if abs(left - right) > 0.05:
            return 1 if left > right else -1
        return 1 if heading >= 0 else -1

    def _bounded(self, linear, angular, scan):
        radius = self.config.robot_radius + self.config.safety_margin
        if not scan.observed(0.0):
            self.pause("front laser sector is unknown")
            return VelocityCommand()
        # A circular footprint permits in-place rotation. Translation is slowed
        # by the measured free travel rather than by centre-to-surface distance.
        free_travel = scan.clearance(0.0, radius, scan.range_max)
        linear = min(linear, max(0.0, 0.8 * (free_travel - 0.02)))
        angular = max(-self.config.max_angular_speed, min(self.config.max_angular_speed, angular))
        return VelocityCommand(max(0.0, min(self.config.max_forward_speed, linear)), angular)

    def step(self, pose, scan, now):
        if self.state in ("DONE", "BLOCKED"):
            return VelocityCommand()
        if not math.isfinite(now) or (self.last_time is not None and now < self.last_time):
            self.state, self.status = "BLOCKED", "clock moved backwards; restart navigation at the start"
            return VelocityCommand()
        self.last_time = now
        goal = self.waypoints[self.goal_index]
        dx, dy = goal.x - pose.x, goal.y - pose.y
        distance = math.hypot(dx, dy)
        heading = wrap_angle(math.atan2(dy, dx) - pose.yaw)
        radius = self.config.robot_radius + self.config.safety_margin
        if scan.nearest <= radius:
            self.pause("obstacle inside robot radius + safety margin")
            return VelocityCommand()
        if not scan.observed(0.0):
            self.pause("front laser sector is unknown")
            return VelocityCommand()

        if distance <= goal.tolerance:
            self.state = "HOLD"
            if self.hold_since is None:
                self.hold_since = now
            elapsed = now - self.hold_since
            self.status = (f"HOLD goal {self.goal_index + 1}/{len(self.waypoints)}: "
                           f"error {distance:.3f} m, {elapsed:.1f}/{self.config.goal_hold_seconds:.1f} s")
            if elapsed + 1e-9 >= self.config.goal_hold_seconds:
                self.goal_index += 1
                self.hold_since = self.follow_since = self.hit_distance = None
                self.turning_away = False
                if self.goal_index == len(self.waypoints):
                    self.state, self.status = "DONE", "DONE: all waypoints held; keeping zero velocity"
                else:
                    self.state = "GO_TO_GOAL"
                    self.status = f"Starting goal {self.goal_index + 1}/{len(self.waypoints)}"
            return VelocityCommand()
        if self.state == "HOLD":
            # Pose drift/noise outside the configured tolerance restarts both
            # approach and the continuous hold interval.
            self.hold_since = self.follow_since = self.hit_distance = None
            self.turning_away = False
            self.state = "GO_TO_GOAL"

        front_clearance = scan.clearance(0.0, radius)
        if self.state == "GO_TO_GOAL":
            # Turn toward a distant goal before interpreting current-front
            # objects as blockers. An obstacle beyond the goal is irrelevant.
            if (abs(heading) < self.config.heading_gate
                    and front_clearance < min(self.config.obstacle_trigger_distance, distance)):
                self.turn_sign = self._turn_choice(scan, heading)
                self.follow_since, self.hit_distance = now, distance
                self.turning_away = False
                self.state = "FOLLOW_WALL"

        if self.state == "FOLLOW_WALL":
            elapsed = now - self.follow_since
            if elapsed > self.config.avoidance_timeout:
                self.state, self.status = "BLOCKED", "avoidance timed out; keeping zero velocity"
                return VelocityCommand()
            # A slightly wider exit corridor provides hysteresis, so we do not
            # leave a wall only to hit the same corner on the very next tick.
            goal_clear = (scan.observed(heading)
                          and distance < scan.range_max - radius
                          and scan.clearance(heading, radius + 0.05, distance + 0.05) >= distance)
            if (elapsed >= self.config.min_wall_follow_seconds
                    and distance <= self.hit_distance - self.config.leave_progress and goal_clear):
                self.state = "GO_TO_GOAL"
                self.follow_since = self.hit_distance = None
                self.turning_away = False
            else:
                side = -self.turn_sign
                side_angle = side * math.pi / 2
                if not scan.observed(side_angle):
                    self.pause("wall-follow laser sector is unknown")
                    return VelocityCommand()
                wall = scan.wall(side, self.config.wall_distance + 0.6)
                # Finish turning past a front obstruction before allowing the
                # side-wall fit to steer again. A single threshold oscillates
                # when a slanted face (or another nearby box) requests the
                # opposite turn on the following tick.
                if front_clearance < 0.20:
                    self.turning_away = True
                elif front_clearance > 0.45:
                    self.turning_away = False
                if self.turning_away:
                    linear, angular = 0.0, self.turn_sign * self.config.max_angular_speed
                elif wall is None:
                    # Convex corner: keep moving slowly while turning toward
                    # the previous wall until it reappears or the goal clears.
                    linear, angular = self.config.wall_follow_speed, side * 0.45
                else:
                    wall_range, tangent = wall
                    angular = (self.config.wall_heading_kp * tangent
                               + side * self.config.wall_kp * (wall_range - self.config.wall_distance))
                    linear = self.config.wall_follow_speed * max(0.0, math.cos(tangent))
                    if abs(tangent) > 0.9:
                        linear = 0.0
                self.status = (f"FOLLOW_WALL goal {self.goal_index + 1}: "
                               f"{'left' if self.turn_sign > 0 else 'right'} bypass, error {distance:.3f} m")
                return self._bounded(linear, angular, scan)

        angular = self.config.angular_kp * heading
        linear = 0.0 if abs(heading) >= self.config.heading_gate else (
            min(self.config.max_forward_speed, self.config.linear_kp * distance)
            * max(0.0, math.cos(heading)))
        self.status = f"GO_TO_GOAL {self.goal_index + 1}/{len(self.waypoints)}: error {distance:.3f} m"
        return self._bounded(linear, angular, scan)

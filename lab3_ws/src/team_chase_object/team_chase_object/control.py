"""ROS-independent, fail-safe distance and bearing control for Lab 3.

Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
Replace the placeholders above before submitting the lab.
"""

from dataclasses import dataclass, field
import math
from typing import Optional


def _finite(value: float) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class PIDConfig:
    """Use P, PI, PD, or PID by setting the unused gains to zero."""

    kp: float
    ki: float = 0.0
    kd: float = 0.0

    def __post_init__(self) -> None:
        for name in ("kp", "ki", "kd"):
            value = getattr(self, name)
            if not _finite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and nonnegative")


@dataclass(frozen=True)
class ChaseConfig:
    target_distance: float = 0.6
    distance_deadband: float = 0.05
    bearing_deadband: float = 0.04
    heading_gate: float = 0.6
    max_forward_speed: float = 0.15
    max_reverse_speed: float = 0.10
    max_angular_speed: float = 0.8
    observation_timeout: float = 0.5
    max_future_stamp: float = 0.1
    min_confidence: float = 0.2
    control_rate_hz: float = 20.0
    base_frame: str = "base_link"
    linear_pid: PIDConfig = field(default_factory=lambda: PIDConfig(kp=0.5))
    angular_pid: PIDConfig = field(default_factory=lambda: PIDConfig(kp=1.5))

    def __post_init__(self) -> None:
        for name in (
            "target_distance", "max_forward_speed", "max_reverse_speed",
            "max_angular_speed", "observation_timeout", "control_rate_hz",
        ):
            value = getattr(self, name)
            if not _finite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("distance_deadband", "bearing_deadband", "max_future_stamp", "min_confidence"):
            value = getattr(self, name)
            if not _finite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.distance_deadband >= self.target_distance:
            raise ValueError("distance_deadband must be smaller than target_distance")
        if not _finite(self.heading_gate) or not 0.0 < self.heading_gate <= math.pi:
            raise ValueError("heading_gate must be in (0, pi] radians")
        if self.bearing_deadband >= self.heading_gate:
            raise ValueError("bearing_deadband must be smaller than heading_gate")
        if self.max_future_stamp > 0.1:
            raise ValueError("max_future_stamp must not exceed 0.1 seconds")
        if self.min_confidence > 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        if not isinstance(self.base_frame, str) or not self.base_frame.strip():
            raise ValueError("base_frame must be a nonempty frame name")
        if not isinstance(self.linear_pid, PIDConfig) or not isinstance(self.angular_pid, PIDConfig):
            raise ValueError("linear_pid and angular_pid must be PIDConfig instances")


@dataclass(frozen=True)
class PolarObservation:
    detected: bool
    bearing: float
    distance: float
    confidence: float
    stamp: float
    frame_id: str = "base_link"


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


class PID:
    """Bounded PID with conditional integration and no initial derivative kick."""

    def __init__(self, config: PIDConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error: Optional[float] = None

    def step(self, error: float, dt: float, lower: float, upper: float) -> float:
        if not all(_finite(x) for x in (error, dt, lower, upper)) or dt <= 0.0 or lower >= upper:
            raise ValueError("PID requires finite inputs, positive dt, and lower < upper")
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        self.previous_error = error
        # Do not accumulate an unused integral for a P or PD controller.
        candidate = self.integral + error * dt if self.config.ki else 0.0
        candidate_output = self.config.kp * error + self.config.ki * candidate + self.config.kd * derivative
        if not math.isfinite(candidate_output):
            self.reset()
            raise ValueError("PID arithmetic overflow")
        # Reject integration only when it would drive saturation farther out.
        if not ((candidate_output > upper and error > 0.0) or (candidate_output < lower and error < 0.0)):
            self.integral = candidate
        output = self.config.kp * error + self.config.ki * self.integral + self.config.kd * derivative
        if not math.isfinite(output):
            self.reset()
            raise ValueError("PID arithmetic overflow")
        return max(lower, min(upper, output))


class ChaseController:
    """Tracks only fresh base-frame observations; missing data always yields zero.

    ``now_ros`` and ``stamp`` use the same ROS time domain. ``now_monotonic``
    comes from a steady clock, so pausing ROS simulated time cannot bypass the
    receive watchdog. Bearing is in radians and positive to the robot's left.
    """

    def __init__(self, config: Optional[ChaseConfig] = None) -> None:
        self.config = config or ChaseConfig()
        self.linear = PID(self.config.linear_pid)
        self.angular = PID(self.config.angular_pid)
        self._observation: Optional[PolarObservation] = None
        self._received_at: Optional[float] = None
        self._last_step: Optional[float] = None
        self._source_watermark: Optional[float] = None
        self._last_ros_time: Optional[float] = None
        self.last_stop_reason = "no observation"

    def reset(self, reason: str = "reset") -> VelocityCommand:
        # Retain timestamp history: stopping must not make old detections new.
        self.linear.reset()
        self.angular.reset()
        self._observation = None
        self._received_at = None
        self._last_step = None
        self.last_stop_reason = reason
        return VelocityCommand()

    def _track_ros_clock(self, now_ros: float) -> bool:
        if not _finite(now_ros):
            return False
        jumped_back = self._last_ros_time is not None and now_ros < self._last_ros_time - 0.1
        self._last_ros_time = now_ros
        if jumped_back:
            self.reset("ROS clock moved backwards")
            # A real /clock reset starts a new timestamp epoch. Merely receiving
            # an older source timestamp never enters this path.
            self._source_watermark = None
        return jumped_back

    def _timestamp_reason(self, observation: PolarObservation, now_ros: float) -> Optional[str]:
        if not _finite(now_ros) or not _finite(observation.stamp):
            return "invalid timestamp"
        if observation.stamp <= 0.0:
            return "missing source timestamp"
        age = now_ros - observation.stamp
        if age < -self.config.max_future_stamp:
            return "source timestamp is in the future"
        if age > self.config.observation_timeout:
            return "source observation is stale"
        return None

    def _invalid_reason(self, observation: PolarObservation, now_ros: float) -> Optional[str]:
        timestamp_reason = self._timestamp_reason(observation, now_ros)
        if timestamp_reason is not None:
            return timestamp_reason
        if not observation.detected:
            return "target not detected"
        if observation.frame_id != self.config.base_frame:
            return "unexpected observation frame"
        if not all(_finite(x) for x in (observation.bearing, observation.distance, observation.confidence)):
            return "nonfinite observation"
        if not -math.pi <= observation.bearing <= math.pi or observation.distance <= 0.0:
            return "invalid bearing or distance"
        if not self.config.min_confidence <= observation.confidence <= 1.0:
            return "invalid or low confidence"
        return None

    def receive(self, observation: PolarObservation, now_ros: float, now_monotonic: float) -> bool:
        """Accept a measurement or reset immediately; return acceptance status."""
        self._track_ros_clock(now_ros)
        reason = self._invalid_reason(observation, now_ros)
        if self._timestamp_reason(observation, now_ros) is None:
            if self._source_watermark is not None and observation.stamp <= self._source_watermark:
                reason = "duplicate or out-of-order source timestamp"
            else:
                # A fresh negative detection is an event too. Otherwise an older
                # positive detection could restart motion after the target is lost.
                self._source_watermark = observation.stamp
        if not _finite(now_monotonic):
            reason = "invalid steady clock"
        if reason is not None:
            self.reset(reason)
            return False
        self._observation = observation
        self._received_at = now_monotonic
        return True

    def step(self, now_ros: float, now_monotonic: float) -> VelocityCommand:
        if self._track_ros_clock(now_ros):
            return VelocityCommand()
        if self._observation is None or self._received_at is None:
            return VelocityCommand()
        if not _finite(now_monotonic):
            return self.reset("invalid steady clock")
        received_age = now_monotonic - self._received_at
        if received_age < 0.0 or received_age > self.config.observation_timeout:
            return self.reset("receive watchdog expired")
        reason = self._invalid_reason(self._observation, now_ros)
        if reason is not None:
            return self.reset(reason)
        dt = 1.0 / self.config.control_rate_hz if self._last_step is None else now_monotonic - self._last_step
        if not _finite(dt) or dt <= 0.0:
            return self.reset("invalid control interval")
        self._last_step = now_monotonic
        distance_error = self._observation.distance - self.config.target_distance
        bearing_error = self._observation.bearing
        try:
            if abs(bearing_error) <= self.config.bearing_deadband + 1e-12:
                self.angular.reset()
                angular_z = 0.0
            else:
                angular_z = self.angular.step(
                    bearing_error, dt, -self.config.max_angular_speed, self.config.max_angular_speed
                )
            if abs(distance_error) <= self.config.distance_deadband + 1e-12 or abs(bearing_error) >= self.config.heading_gate:
                self.linear.reset()
                linear_x = 0.0
            else:
                linear_x = self.linear.step(
                    distance_error, dt, -self.config.max_reverse_speed, self.config.max_forward_speed
                )
        except ValueError:
            return self.reset("invalid PID calculation")
        self.last_stop_reason = ""
        return VelocityCommand(linear_x=linear_x, angular_z=angular_z)

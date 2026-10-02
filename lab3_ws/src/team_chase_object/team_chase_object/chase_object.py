"""Lab 3 ROS 2 node: polar target observations -> safe bounded cmd_vel.

Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
Replace the placeholders above before submitting the lab.
"""

import signal
import threading

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist, TwistStamped
from chase_object_interfaces.msg import ObjectPolar

from .control import ChaseConfig, ChaseController, PIDConfig, PolarObservation, VelocityCommand


class ChaseObjectNode(Node):
    def __init__(self) -> None:
        super().__init__("chase_object")

        def parameter(name, default, description):
            return self.declare_parameter(
                name, default, ParameterDescriptor(description=description, read_only=True)
            ).value

        config = ChaseConfig(
            target_distance=parameter("target_distance", 0.6, "Desired object range in metres."),
            distance_deadband=parameter("distance_deadband", 0.05, "Distance tolerance in metres; commands exactly zero inside it."),
            bearing_deadband=parameter("bearing_deadband", 0.04, "Bearing tolerance in radians; commands exactly zero inside it."),
            heading_gate=parameter("heading_gate", 0.6, "Suppress translation when absolute bearing is at least this angle, in radians."),
            max_forward_speed=parameter("max_forward_speed", 0.25, "Maximum forward velocity in m/s."),
            max_reverse_speed=parameter("max_reverse_speed", 0.15, "Maximum reverse speed magnitude in m/s."),
            max_angular_speed=parameter("max_angular_speed", 0.8, "Maximum angular speed magnitude in rad/s."),
            observation_timeout=parameter("observation_timeout", 0.5, "Maximum source and receipt age in seconds."),
            max_future_stamp=parameter("max_future_stamp", 0.1, "Allowed future timestamp tolerance, at most 0.1 seconds."),
            min_confidence=parameter("min_confidence", 0.2, "Minimum accepted target confidence in [0,1]."),
            control_rate_hz=parameter("control_rate_hz", 20.0, "Steady-clock command publication frequency in Hz."),
            base_frame=parameter("base_frame", "base_link", "Required frame for bearing and range."),
            linear_pid=PIDConfig(
                kp=parameter("linear_kp", 0.7, "Distance proportional gain."),
                ki=parameter("linear_ki", 0.0, "Distance integral gain; zero disables integral term."),
                kd=parameter("linear_kd", 0.0, "Distance derivative gain; zero disables derivative term."),
            ),
            angular_pid=PIDConfig(
                kp=parameter("angular_kp", 1.5, "Bearing proportional gain."),
                ki=parameter("angular_ki", 0.0, "Bearing integral gain; zero disables integral term."),
                kd=parameter("angular_kd", 0.0, "Bearing derivative gain; zero disables derivative term."),
            ),
        )
        polar_topic = parameter("polar_topic", "/object/polar", "Input ObjectPolar topic.")
        cmd_vel_topic = parameter("cmd_vel_topic", "/cmd_vel", "Output velocity topic.")
        self._stamped = parameter("cmd_vel_stamped", False, "Publish TwistStamped when required by the robot base.")
        self._controller = ChaseController(config)
        self._base_frame = config.base_frame
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._stopping = False
        self._publisher = self.create_publisher(TwistStamped if self._stamped else Twist, cmd_vel_topic, 10)
        self._subscription = self.create_subscription(
            ObjectPolar, polar_topic, self._on_observation, qos_profile_sensor_data
        )
        self._timer = self.create_timer(
            1.0 / config.control_rate_hz, self._on_timer, clock=self._steady_clock
        )
        self.get_logger().info(
            f"Listening on {polar_topic}; publishing {'TwistStamped' if self._stamped else 'Twist'} "
            f"on {cmd_vel_topic}; target distance {config.target_distance:.2f} m."
        )

    def _times(self):
        return (
            self.get_clock().now().nanoseconds * 1e-9,
            self._steady_clock.now().nanoseconds * 1e-9,
        )

    def _on_observation(self, message: ObjectPolar) -> None:
        if self._stopping:
            return
        observation = PolarObservation(
            detected=message.detected,
            bearing=message.bearing,
            distance=message.distance,
            confidence=message.confidence,
            stamp=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            frame_id=message.header.frame_id,
        )
        now_ros, now_monotonic = self._times()
        if not self._controller.receive(observation, now_ros, now_monotonic):
            # Invalid data must not leave the preceding nonzero command active.
            self._publish(VelocityCommand())

    def _on_timer(self) -> None:
        if self._stopping:
            return
        now_ros, now_monotonic = self._times()
        self._publish(self._controller.step(now_ros, now_monotonic))

    def _publish(self, command: VelocityCommand) -> None:
        if self._stamped:
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self._base_frame
            twist = message.twist
        else:
            message = Twist()
            twist = message
        twist.linear.x = command.linear_x
        twist.angular.z = command.angular_z
        self._publisher.publish(message)

    def stop(self) -> None:
        self._stopping = True
        self._timer.cancel()
        self._publish(self._controller.reset("shutdown"))
        # Give reliable subscribers a bounded opportunity to receive the stop.
        try:
            self._publisher.wait_for_all_acked(Duration(seconds=0.2))
        except NotImplementedError:
            self.get_logger().warning("The selected RMW cannot wait for stop-message acknowledgements.")


def main(args=None) -> None:
    # Handle termination before shutting down the ROS context, allowing a final
    # stop to be published. A base-side velocity timeout is still recommended.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stop_requested = threading.Event()
    old_handlers = {}

    def request_stop(_signum, _frame):
        stop_requested.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, request_stop)
        node = ChaseObjectNode()
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                try:
                    if rclpy.ok():
                        node.stop()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            for signum, old_handler in old_handlers.items():
                signal.signal(signum, old_handler)


if __name__ == "__main__":
    main()

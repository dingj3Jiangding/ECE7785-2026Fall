"""Lab 4 ROS 2 navigation node: odometry and LiDAR to bounded velocity.

Lab partners: Ding Jiang, Tongning Zhang.
"""

import json
import math
from pathlib import Path
import signal
import threading

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import RigidTransform, StartFrame
from .navigation import NavigationConfig, Navigator, VelocityCommand, load_waypoints
from .scan import ScanConfig, process_scan
from .watchdog import FreshnessGuard


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def normalized_quaternion(values):
    """Validate quaternions before applying ROS TF or odometry transforms."""
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid zero or overflowing quaternion")
    return tuple(value / norm for value in values)


def quaternion_product(left, right):
    x, y, z, w = left
    a, b, c, d = right
    return normalized_quaternion((
        w * a + x * d + y * c - z * b,
        w * b - x * c + y * d + z * a,
        w * c + x * b - y * a + z * d,
        w * d - x * a - y * b - z * c,
    ))


def rotate_vector(quaternion, vector):
    x, y, z, w = quaternion
    a, b, c = vector
    # q * (v, 0) * inverse(q), for a unit quaternion.
    tx, ty, tz = 2.0 * (y * c - z * b), 2.0 * (z * a - x * c), 2.0 * (x * b - y * a)
    return (
        a + w * tx + y * tz - z * ty,
        b + w * ty + z * tx - x * tz,
        c + w * tz + x * ty - y * tx,
    )


class NavigateToGoal(Node):
    def __init__(self):
        super().__init__("navigate_to_goal")

        def parameter(name, default, description):
            return self.declare_parameter(
                name, default,
                ParameterDescriptor(description=description, read_only=True),
            ).value

        self.base_frame = parameter("base_frame", "base_link", "Robot frame used for control and the circular footprint.")
        self.pose_frame = parameter("pose_frame", "lab_start", "Debug pose frame: first valid robot pose defines its origin.")
        odom_topic = parameter("odom_topic", "/odom", "Input nav_msgs/Odometry topic.")
        scan_topic = parameter("scan_topic", "/scan", "Input sensor_msgs/LaserScan topic.")
        cmd_topic = parameter("cmd_vel_topic", "/cmd_vel", "Output velocity topic.")
        pose_topic = parameter("pose_topic", "/navigation/pose", "Debug pose topic; no TF is broadcast.")
        state_topic = parameter("state_topic", "/navigation/state", "JSON navigation state in std_msgs/String.")
        self.stamped = parameter("cmd_vel_stamped", False, "Publish TwistStamped instead of Twist if required by the base.")
        control_rate = parameter("control_rate_hz", 20.0, "Steady-clock command publication frequency in Hz.")
        pose_timeout = parameter("pose_timeout", 0.5, "Maximum odometry source and receive age in seconds.")
        scan_timeout = parameter("scan_timeout", 0.5, "Maximum LiDAR source and receive age in seconds.")
        future_tolerance = parameter("max_future_stamp", 0.1, "Maximum tolerated future source timestamp in seconds.")
        default_waypoints = str(Path(get_package_share_directory("team_navigate_to_goal")) / "config" / "wayPoints.txt")
        waypoint_file = parameter("waypoints_file", "", "Waypoint file; empty uses the installed config/wayPoints.txt.")
        tolerances = parameter("goal_tolerances", [0.08, 0.12, 0.16], "Arrival radius in metres for each waypoint.")
        config = NavigationConfig(
            max_forward_speed=parameter("max_forward_speed", 0.15, "Maximum forward speed in m/s."),
            max_angular_speed=parameter("max_angular_speed", 0.8, "Maximum angular speed in rad/s."),
            linear_kp=parameter("linear_kp", 0.5, "Proportional gain for distance to goal."),
            angular_kp=parameter("angular_kp", 1.5, "Proportional gain for heading to goal."),
            heading_gate=parameter("heading_gate", 0.6, "Pause forward travel above this heading error in radians."),
            goal_hold_seconds=parameter("goal_hold_seconds", 10.0, "Stopped dwell time at each goal in ROS seconds."),
            robot_radius=parameter("robot_radius", 0.22, "Measured circular robot footprint radius in metres; default is a conservative starting estimate."),
            safety_margin=parameter("safety_margin", 0.04, "Extra clearance beyond the robot footprint in metres."),
            obstacle_trigger_distance=parameter("obstacle_trigger_distance", 0.65, "Forward path lookahead for starting obstacle avoidance in metres."),
            wall_distance=parameter("wall_distance", 0.40, "Desired base-centre to obstacle surface distance while following a wall, in metres."),
            wall_follow_speed=parameter("wall_follow_speed", 0.10, "Maximum wall-following speed in m/s."),
            wall_kp=parameter("wall_kp", 2.0, "Wall-distance correction gain."),
            wall_heading_kp=parameter("wall_heading_kp", 1.5, "Wall-heading correction gain."),
            leave_progress=parameter("leave_progress", 0.10, "Minimum goal-distance improvement before leaving a wall in metres."),
            min_wall_follow_seconds=parameter("min_wall_follow_seconds", 1.0, "Minimum time in wall-follow mode before trying to leave."),
            avoidance_timeout=parameter("avoidance_timeout", 60.0, "Stop if one obstacle-avoidance attempt exceeds this many ROS seconds."),
            preferred_turn=parameter("preferred_turn", "auto", "Initial avoidance side: auto, left, or right."),
        )
        self.scan_config = ScanConfig(
            min_valid_fraction=parameter("min_valid_scan_fraction", 0.5, "Minimum usable fraction of LaserScan measurements."),
        )
        if not self.base_frame or not self.pose_frame:
            raise ValueError("base_frame and pose_frame must be nonempty")
        if not math.isfinite(control_rate) or control_rate <= 0:
            raise ValueError("control_rate_hz must be finite and positive")
        self.pose_guard = FreshnessGuard(timeout=pose_timeout, max_future_stamp=future_tolerance)
        self.scan_guard = FreshnessGuard(timeout=scan_timeout, max_future_stamp=future_tolerance)
        self.navigator = Navigator(load_waypoints(waypoint_file or default_waypoints, tolerances=tolerances), config)
        self.start_frame = StartFrame()
        self.pose = None
        self.scan = None
        self.odom_frame = None
        self.odom_child_frame = None
        self.last_ros_time = None
        self.fault_reason = None
        self.stopping = False
        self.last_state_message = None
        self.last_state_publication = None
        self.last_state_key = None
        self.last_state_log = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.velocity_pub = self.create_publisher(TwistStamped if self.stamped else Twist, cmd_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / control_rate, self.on_timer, clock=self.steady_clock)
        self.get_logger().info(
            f"Inputs: {odom_topic}, {scan_topic}; output: {cmd_topic} "
            f"({'TwistStamped' if self.stamped else 'Twist'}). "
            "The first valid odometry pose defines the lab origin."
        )
        self.publish_state("WAITING_FOR_DATA", "Waiting for fresh odometry and LiDAR with valid TF.")

    def times(self):
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        now_steady = self.steady_clock.now().nanoseconds * 1e-9
        if self.last_ros_time is not None and now_ros < self.last_ros_time - 0.1:
            self.fault_reason = "ROS clock moved backwards; restart the node at the lab start position."
            self.pose_guard.invalidate(self.fault_reason)
            self.scan_guard.invalidate(self.fault_reason)
            self.navigator.pause(self.fault_reason)
        self.last_ros_time = now_ros
        return now_ros, now_steady

    def transform(self, target, source, stamp):
        if target == source:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        transform = self.tf_buffer.lookup_transform(
            target, source, Time.from_msg(stamp), timeout=Duration(seconds=0.0),
        ).transform
        translation = (transform.translation.x, transform.translation.y, transform.translation.z)
        rotation = normalized_quaternion((
            transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w,
        ))
        if not all(math.isfinite(value) for value in translation):
            raise ValueError("TF translation contains nonfinite values")
        return translation, rotation

    def halt(self, reason, state="WAITING_FOR_DATA"):
        self.navigator.pause(reason)
        self.publish_velocity(VelocityCommand())
        self.publish_state(state, reason)

    def on_odom(self, message):
        if self.stopping:
            return
        now_ros, now_steady = self.times()
        if self.fault_reason:
            self.halt(self.fault_reason, "FAULT")
            return
        if not self.pose_guard.receive(stamp_seconds(message.header.stamp), now_ros, now_steady):
            self.halt("Odometry: " + self.pose_guard.reason)
            return
        try:
            frame = message.header.frame_id
            child = message.child_frame_id
            if not frame or child not in (self.base_frame, "base_footprint"):
                raise ValueError("odometry requires a nonempty world frame and child base_frame or base_footprint")
            if self.odom_frame is not None and (frame != self.odom_frame or child != self.odom_child_frame):
                raise ValueError("odometry frame IDs changed; restore the original frames or restart at the start position")
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            xyz = (position.x, position.y, position.z)
            if not all(math.isfinite(value) for value in xyz):
                raise ValueError("odometry position contains nonfinite values")
            quaternion = normalized_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
            # Odometry gives T_world_child. Compose T_child_base so the start
            # origin, waypoint distances, and scan footprint share base_frame.
            translation, rotation = self.transform(child, self.base_frame, message.header.stamp)
            offset = rotate_vector(quaternion, translation)
            quaternion = quaternion_product(quaternion, rotation)
            pose = self.start_frame.update(xyz[0] + offset[0], xyz[1] + offset[1], quaternion=quaternion)
        except (ValueError, TransformException) as exc:
            self.pose_guard.invalidate(str(exc))
            self.halt("Odometry: " + str(exc))
            return
        self.odom_frame, self.odom_child_frame = frame, child
        self.pose = pose
        debug_pose = PoseStamped()
        debug_pose.header.stamp = message.header.stamp
        debug_pose.header.frame_id = self.pose_frame
        debug_pose.pose.position.x, debug_pose.pose.position.y = pose.x, pose.y
        debug_pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        debug_pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        self.pose_pub.publish(debug_pose)

    def on_scan(self, message):
        if self.stopping:
            return
        now_ros, now_steady = self.times()
        if self.fault_reason:
            self.halt(self.fault_reason, "FAULT")
            return
        if not self.scan_guard.receive(stamp_seconds(message.header.stamp), now_ros, now_steady):
            self.halt("LiDAR: " + self.scan_guard.reason)
            return
        try:
            if not message.header.frame_id:
                raise ValueError("LaserScan frame_id must be nonempty")
            translation, rotation = self.transform(self.base_frame, message.header.frame_id, message.header.stamp)
            self.scan = process_scan(
                message.ranges, message.angle_min, message.angle_increment,
                message.range_min, message.range_max,
                transform=RigidTransform(translation=translation, rotation=rotation),
                config=self.scan_config,
            )
        except (ValueError, TransformException) as exc:
            self.scan_guard.invalidate(str(exc))
            self.halt("LiDAR: " + str(exc))

    def on_timer(self):
        if self.stopping:
            return
        now_ros, now_steady = self.times()
        if self.fault_reason:
            self.halt(self.fault_reason, "FAULT")
            return
        if self.pose is None or not self.pose_guard.fresh(now_ros, now_steady):
            self.halt("Odometry: " + self.pose_guard.reason)
            return
        if self.scan is None or not self.scan_guard.fresh(now_ros, now_steady):
            self.halt("LiDAR: " + self.scan_guard.reason)
            return
        try:
            command = self.navigator.step(self.pose, self.scan, now_ros)
        except ValueError as exc:
            self.fault_reason = "Navigation calculation failed: " + str(exc)
            self.halt(self.fault_reason, "FAULT")
            return
        self.publish_velocity(command)
        self.publish_state()

    def publish_velocity(self, command):
        if self.stamped:
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.base_frame
            twist = message.twist
        else:
            message = Twist()
            twist = message
        twist.linear.x = command.linear_x
        twist.angular.z = command.angular_z
        self.velocity_pub.publish(message)

    def publish_state(self, state=None, status=None):
        state = self.navigator.state if state is None else state
        key = (state, self.navigator.goal_index)
        payload = json.dumps({
            "state": state,
            "goal_index": self.navigator.goal_index,
            "status": self.navigator.status if status is None else status,
        }, ensure_ascii=False)
        now = self.steady_clock.now().nanoseconds * 1e-9
        changed = key != self.last_state_key
        if changed or self.last_state_publication is None or now - self.last_state_publication >= 1.0:
            message = String()
            message.data = payload
            self.state_pub.publish(message)
            self.last_state_message = payload
            self.last_state_publication = now
        if changed or self.last_state_log is None or now - self.last_state_log >= 2.0:
            self.get_logger().info(payload)
            self.last_state_log = now
        self.last_state_key = key

    def stop(self):
        self.stopping = True
        self.timer.cancel()
        self.navigator.pause("shutdown")
        self.publish_velocity(VelocityCommand())
        self.publish_state("STOPPED", "Node shutting down.")
        try:
            self.velocity_pub.wait_for_all_acked(Duration(seconds=0.2))
        except NotImplementedError:
            self.get_logger().warning("The selected RMW cannot wait for stop-message acknowledgements.")


def main(args=None):
    # Keep the ROS context alive while sending the last stop command.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stop_requested = threading.Event()
    old_handlers = {}

    def request_stop(_signum, _frame):
        stop_requested.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, request_stop)
        node = NavigateToGoal()
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

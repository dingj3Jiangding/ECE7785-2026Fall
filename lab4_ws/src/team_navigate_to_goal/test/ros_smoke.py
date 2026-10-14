"""Exercise the actual ROS 2 wrapper with synthetic sensors and isolated topics.

Lab partners: Ding Jiang, Tongning Zhang.

Run on Ubuntu after building and sourcing the workspace:
    python3 src/team_navigate_to_goal/test/ros_smoke.py
    python3 src/team_navigate_to_goal/test/ros_smoke.py --stamped

Every data and TF topic is remapped into a fresh UUID namespace. This script
does not publish to the robot's /cmd_vel and does not require a robot. It checks
ROS integration, not real-world navigation or the laboratory demonstration.
"""

import argparse
import json
import math
import sys
import time
import uuid

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster
    from team_navigate_to_goal.navigate_to_goal import NavigateToGoal
except ImportError as exc:
    raise SystemExit(
        f"Missing ROS 2 workspace dependency: {exc}. Source ROS 2 and "
        "install/setup.bash after colcon build."
    ) from exc


BASE_FRAME = "base_link"
FOOTPRINT_FRAME = "base_footprint"
SCAN_FRAME = "base_scan"
CHILD_YAW = 0.3
BASE_YAW_OFFSET = 0.2
BASE_OFFSET = (0.10, -0.03, 0.05)
SENSOR_TIMEOUT = 0.5


def yaw_quaternion(target, yaw):
    target.z, target.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Probe(Node):
    def __init__(self, stamped):
        super().__init__("lab4_smoke_probe")
        self.stamped = stamped
        self.commands = []
        self.poses = []
        self.states = []
        self.odom_pub = self.create_publisher(Odometry, "/odom", qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.create_subscription(TwistStamped if stamped else Twist, "/cmd_vel", self.on_command, 10)
        self.create_subscription(PoseStamped, "/navigation/pose", self.on_pose, 10)
        self.create_subscription(String, "/navigation/state", self.on_state, 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

    def on_command(self, message):
        if self.stamped:
            assert message.header.frame_id == BASE_FRAME, "TwistStamped has wrong frame"
            assert message.header.stamp.sec > 0, "TwistStamped has no source stamp"
        twist = message.twist if self.stamped else message
        assert math.isfinite(twist.linear.x) and math.isfinite(twist.angular.z)
        assert 0.0 <= twist.linear.x <= 0.15 + 1e-9, "Forward speed limit violated"
        assert abs(twist.angular.z) <= 0.8 + 1e-9, "Angular speed limit violated"
        self.commands.append((time.monotonic(), twist))

    def on_pose(self, message):
        self.poses.append((time.monotonic(), message))

    def on_state(self, message):
        self.states.append((time.monotonic(), json.loads(message.data)))

    def publish_tf(self):
        footprint_to_base = TransformStamped()
        footprint_to_base.header.stamp = self.get_clock().now().to_msg()
        footprint_to_base.header.frame_id = FOOTPRINT_FRAME
        footprint_to_base.child_frame_id = BASE_FRAME
        translation = footprint_to_base.transform.translation
        translation.x, translation.y, translation.z = BASE_OFFSET
        yaw_quaternion(footprint_to_base.transform.rotation, BASE_YAW_OFFSET)

        base_to_scan = TransformStamped()
        base_to_scan.header.stamp = footprint_to_base.header.stamp
        base_to_scan.header.frame_id = BASE_FRAME
        base_to_scan.child_frame_id = SCAN_FRAME
        translation = base_to_scan.transform.translation
        translation.x, translation.y, translation.z = 0.12, 0.02, 0.10
        yaw_quaternion(base_to_scan.transform.rotation, 0.15)
        self.tf_broadcaster.sendTransform([footprint_to_base, base_to_scan])

    def publish_sensors(self, *, odom=True, scan=True, yaw_delta=0.0, obstacle=False):
        stamp = self.get_clock().now().to_msg()
        if odom:
            message = Odometry()
            message.header.stamp = stamp
            message.header.frame_id = "smoke_odom"
            message.child_frame_id = FOOTPRINT_FRAME
            message.pose.pose.position.x, message.pose.pose.position.y = 5.0, -2.0
            yaw_quaternion(message.pose.pose.orientation, CHILD_YAW + yaw_delta)
            self.odom_pub.publish(message)
        if scan:
            message = LaserScan()
            message.header.stamp = stamp
            message.header.frame_id = SCAN_FRAME
            message.angle_min = -math.pi
            message.angle_increment = math.pi / 180.0
            message.angle_max = message.angle_min + 359 * message.angle_increment
            message.range_min, message.range_max = 0.05, 8.0
            message.ranges = [math.inf] * 360
            if obstacle:
                # In the scan frame this return is behind the robot at 0.30 m.
                # The nonzero laser-to-base TF puts it about 0.18 m from the
                # base origin: inside the 0.26 m footprint + safety envelope.
                # Ignoring that translation would incorrectly allow driving.
                message.ranges[0] = 0.30
            self.scan_pub.publish(message)

    def forward_since(self, since):
        return bool(self.commands and self.commands[-1][0] >= since
                    and self.commands[-1][1].linear.x > 0.02)

    def stopped_since(self, since):
        return len(self.commands) >= 3 and all(
            received >= since and message.linear.x == 0.0 and message.angular.z == 0.0
            for received, message in self.commands[-3:]
        )

    def status_since(self, since, text):
        return bool(self.states and self.states[-1][0] >= since
                    and text in self.states[-1][1]["status"])

    def pose_matches(self, since, x, y, yaw):
        if not self.poses or self.poses[-1][0] < since:
            return False
        message = self.poses[-1][1]
        pose = message.pose
        orientation = pose.orientation
        observed_yaw = math.atan2(2 * orientation.w * orientation.z,
                                  1 - 2 * orientation.z * orientation.z)
        return (message.header.frame_id == "lab_start"
                and abs(pose.position.x - x) < 1e-5
                and abs(pose.position.y - y) < 1e-5
                and abs(observed_yaw - yaw) < 1e-5)


def spin_until(executor, predicate, publish, description, timeout=8.0):
    deadline = time.monotonic() + timeout
    next_publish = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_publish:
            publish()
            next_publish = now + 0.1
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError(f"Timed out waiting for {description}")


def run(stamped=False):
    namespace = f"/lab4_smoke_{uuid.uuid4().hex[:10]}"
    topics = ("/odom", "/scan", "/cmd_vel", "/navigation/pose", "/navigation/state", "/tf", "/tf_static")
    ros_args = ["--ros-args", "-r", f"__ns:={namespace}",
                "-p", f"cmd_vel_stamped:={'true' if stamped else 'false'}",
                "-p", "use_sim_time:=false"]
    for topic in topics:
        ros_args += ["-r", f"{topic}:={namespace}{topic}"]
    rclpy.init(args=ros_args)
    executor = SingleThreadedExecutor()
    nodes = []
    controller = None
    try:
        controller = NavigateToGoal()
        nodes.append(controller)
        probe = Probe(stamped)
        nodes.append(probe)
        assert controller.velocity_pub.topic_name == namespace + "/cmd_vel", "Velocity topic is not isolated"
        assert controller.odom_sub.topic_name == namespace + "/odom", "Odometry topic is not isolated"
        assert controller.scan_sub.topic_name == namespace + "/scan", "LiDAR topic is not isolated"
        for node in nodes:
            executor.add_node(node)
        probe.publish_tf()

        phase = time.monotonic()
        spin_until(executor,
                   lambda: probe.forward_since(phase) and probe.pose_matches(phase, 0.0, 0.0, 0.0),
                   probe.publish_sensors, "initial TF, start-frame pose, and forward velocity")
        print("PASS: odometry + scan + TF -> start-frame pose and bounded forward velocity")

        # Rotating base_footprint about a fixed point moves the offset base_link
        # origin. This checks translation composition rather than just TF lookup.
        yaw_delta = 0.5
        a, b = BASE_OFFSET[:2]
        old_x = math.cos(CHILD_YAW) * a - math.sin(CHILD_YAW) * b
        old_y = math.sin(CHILD_YAW) * a + math.cos(CHILD_YAW) * b
        new_x = math.cos(CHILD_YAW + yaw_delta) * a - math.sin(CHILD_YAW + yaw_delta) * b
        new_y = math.sin(CHILD_YAW + yaw_delta) * a + math.cos(CHILD_YAW + yaw_delta) * b
        c, s = math.cos(CHILD_YAW + BASE_YAW_OFFSET), math.sin(CHILD_YAW + BASE_YAW_OFFSET)
        expected_x, expected_y = c * (new_x - old_x) + s * (new_y - old_y), -s * (new_x - old_x) + c * (new_y - old_y)
        phase = time.monotonic()
        spin_until(executor, lambda: probe.pose_matches(phase, expected_x, expected_y, yaw_delta),
                   lambda: probe.publish_sensors(yaw_delta=yaw_delta), "composed nonzero odometry-frame TF")
        print("PASS: base_footprint -> base_link nonzero translation and rotation are composed")

        phase = time.monotonic()
        spin_until(executor,
                   lambda: probe.forward_since(phase) and probe.pose_matches(phase, 0.0, 0.0, 0.0),
                   probe.publish_sensors, "forward motion after restoring the synthetic pose")
        phase = time.monotonic()
        spin_until(executor,
                   lambda: probe.stopped_since(phase) and probe.status_since(phase, "obstacle inside"),
                   lambda: probe.publish_sensors(obstacle=True), "stop for a transformed close obstacle")
        print("PASS: nonzero laser-to-base TF + close obstacle -> zero linear and angular velocity")

        phase = time.monotonic()
        spin_until(executor, lambda: probe.forward_since(phase), probe.publish_sensors,
                   "forward motion after obstacle removal")
        print("PASS: clear scan -> forward motion resumes")

        for missing, publisher in (
            ("LiDAR", lambda: probe.publish_sensors(scan=False)),
            ("Odometry", lambda: probe.publish_sensors(odom=False)),
        ):
            phase = time.monotonic()
            spin_until(
                executor,
                lambda: (time.monotonic() - phase > SENSOR_TIMEOUT + 0.1
                         and probe.stopped_since(phase)
                         and probe.status_since(phase, missing + ": sensor watchdog expired")),
                publisher, f"{missing} dropout watchdog stop", timeout=4.0,
            )
            print(f"PASS: {missing} messages stop while the other sensor continues -> watchdog stop")
            phase = time.monotonic()
            spin_until(executor, lambda: probe.forward_since(phase), probe.publish_sensors,
                       f"forward motion after {missing} recovery")
            print(f"PASS: fresh {missing} messages -> forward motion resumes")

        print(f"ROS 2 smoke test passed ({'TwistStamped' if stamped else 'Twist'}) in {namespace}")
    finally:
        if controller is not None:
            try:
                controller.stop()
            except Exception as exc:
                print(f"Cleanup stop failed: {exc}", file=sys.stderr)
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stamped", action="store_true", help="Exercise the optional TwistStamped velocity interface.")
    arguments = parser.parse_args()
    try:
        run(stamped=arguments.stamped)
    except (AssertionError, RuntimeError, KeyboardInterrupt) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)

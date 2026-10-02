"""Run the three ROS 2 nodes against synthetic sensors on isolated topics.

Usage on Ubuntu, after building and sourcing this workspace:
    python3 src/team_chase_object/test/ros_smoke.py

This checks ROS message wiring, TF, image decoding, sensor fusion, control, and
loss-of-target stopping. It does not validate a real robot or the 5-second demo.
"""

import math
import sys
import time
import uuid

try:
    import cv2
    import numpy as np
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
    from tf2_ros import StaticTransformBroadcaster
    from chase_object_interfaces.msg import ObjectBearing, ObjectPolar
    from team_chase_object.chase_object import ChaseObjectNode
    from team_chase_object.detect_object import DetectObject
    from team_chase_object.get_object_range import GetObjectRange
except ImportError as exc:
    raise SystemExit(
        f"Missing ROS 2 workspace dependency: {exc}. Source ROS 2 and "
        "install/setup.bash after colcon build."
    ) from exc


WIDTH, HEIGHT = 640, 480
CAMERA_FRAME = "camera_optical_frame"
LASER_FRAME = "laser_frame"
BASE_FRAME = "base_link"


class Probe(Node):
    def __init__(self):
        super().__init__("lab3_smoke_probe")
        self.bearings = []
        self.polars = []
        self.commands = []
        self.image_pub = self.create_publisher(
            CompressedImage, "/camera/image_raw/compressed", qos_profile_sensor_data
        )
        self.info_pub = self.create_publisher(
            CameraInfo, "/camera/camera_info", qos_profile_sensor_data
        )
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.create_subscription(ObjectBearing, "/object/bearing", self.bearings.append, 10)
        self.create_subscription(ObjectPolar, "/object/polar", self.polars.append, 10)
        self.create_subscription(Twist, "/cmd_vel", self.commands.append, 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.blue_jpeg = self._jpeg(with_target=True)
        self.blank_jpeg = self._jpeg(with_target=False)

    @staticmethod
    def _jpeg(with_target):
        image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        if with_target:
            cv2.rectangle(image, (280, 170), (360, 310), (255, 0, 0), -1)
        ok, encoded = cv2.imencode(".jpg", image)
        if not ok:
            raise RuntimeError("Could not encode synthetic image")
        return encoded.tobytes()

    def publish_tf(self):
        stamp = self.get_clock().now().to_msg()
        base_to_laser = TransformStamped()
        base_to_laser.header.stamp = stamp
        base_to_laser.header.frame_id = BASE_FRAME
        base_to_laser.child_frame_id = LASER_FRAME
        base_to_laser.transform.rotation.w = 1.0

        laser_to_camera = TransformStamped()
        laser_to_camera.header.stamp = stamp
        laser_to_camera.header.frame_id = LASER_FRAME
        laser_to_camera.child_frame_id = CAMERA_FRAME
        # Parent->child optical orientation; lookup(camera, laser) gives
        # (x, y, z, w) = (0.5, -0.5, 0.5, 0.5).
        rotation = laser_to_camera.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = -0.5, 0.5, -0.5, 0.5
        self.tf_broadcaster.sendTransform([base_to_laser, laser_to_camera])

    def publish_sensors(self, with_target):
        stamp = self.get_clock().now().to_msg()

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = CAMERA_FRAME
        info.width, info.height = WIDTH, HEIGHT
        info.k = [400.0, 0.0, 319.5, 0.0, 400.0, 239.5, 0.0, 0.0, 1.0]
        self.info_pub.publish(info)

        image = CompressedImage()
        image.header.stamp = stamp
        image.header.frame_id = CAMERA_FRAME
        image.format = "jpeg"
        image.data = self.blue_jpeg if with_target else self.blank_jpeg
        self.image_pub.publish(image)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = LASER_FRAME
        scan.angle_min = -math.pi
        scan.angle_increment = math.pi / 180.0
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.range_min, scan.range_max = 0.05, 8.0
        scan.ranges = [
            1.0 if with_target and abs(scan.angle_min + i * scan.angle_increment) < 0.05
            else 3.0
            for i in range(360)
        ]
        self.scan_pub.publish(scan)


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


def run():
    suffix = uuid.uuid4().hex[:8]
    namespace = f"/lab3_smoke_{suffix}"
    topics = (
        "/camera/image_raw/compressed", "/camera/camera_info", "/scan",
        "/object/bearing", "/object/polar", "/cmd_vel", "/tf", "/tf_static",
    )
    ros_args = ["--ros-args", "-r", f"__ns:={namespace}"]
    for topic in topics:
        ros_args += ["-r", f"{topic}:={namespace}{topic}"]
    rclpy.init(args=ros_args)
    executor = SingleThreadedExecutor()
    nodes = []
    controller = None
    try:
        detector = DetectObject()
        ranger = GetObjectRange()
        controller = ChaseObjectNode()
        probe = Probe()
        nodes = [detector, ranger, controller, probe]
        for node in nodes:
            executor.add_node(node)
        probe.publish_tf()

        def positive_result():
            return (
                any(message.detected for message in probe.bearings)
                and any(
                    message.detected and message.header.frame_id == BASE_FRAME
                    and 0.85 < message.distance < 1.15
                    and abs(message.bearing) < 0.10
                    for message in probe.polars
                )
                and any(message.linear.x > 0.0 for message in probe.commands)
            )

        spin_until(
            executor, positive_result, lambda: probe.publish_sensors(True),
            "detected target, range, and forward command",
        )
        print("PASS: image -> bearing -> LiDAR range -> forward velocity")

        timeout_command_start = len(probe.commands)
        spin_until(
            executor,
            lambda: any(
                message.linear.x == 0.0 and message.angular.z == 0.0
                for message in probe.commands[timeout_command_start:]
            ),
            lambda: None,
            "zero velocity after sensor messages stop",
            timeout=3.0,
        )
        print("PASS: sensor dropout -> watchdog stop")

        reacquire_command_start = len(probe.commands)
        spin_until(
            executor,
            lambda: any(
                message.linear.x > 0.0
                for message in probe.commands[reacquire_command_start:]
            ),
            lambda: probe.publish_sensors(True),
            "forward command after target reacquisition",
        )

        bearing_start = len(probe.bearings)
        polar_start = len(probe.polars)
        command_start = len(probe.commands)

        def stopped_result():
            return (
                any(not message.detected for message in probe.bearings[bearing_start:])
                and any(not message.detected for message in probe.polars[polar_start:])
                and any(
                    message.linear.x == 0.0 and message.angular.z == 0.0
                    for message in probe.commands[command_start:]
                )
            )

        spin_until(
            executor, stopped_result, lambda: probe.publish_sensors(False),
            "invalid target observation and zero velocity",
        )
        print("PASS: lost target -> invalid polar observation -> zero velocity")
        print(f"ROS 2 smoke test passed in isolated namespace {namespace}")
    finally:
        if controller is not None:
            try:
                controller.stop()
            except Exception:
                pass
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        run()
    except (AssertionError, RuntimeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)

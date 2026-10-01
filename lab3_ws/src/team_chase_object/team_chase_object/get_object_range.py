"""Fuse timestamp-matched target bearings and laser scans using measured TF.

Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
"""
import math

import message_filters
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from chase_object_interfaces.msg import ObjectBearing, ObjectPolar

from .fusion import BearingGate, FusionConfig, RigidTransform, estimate_range


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class GetObjectRange(Node):
    def __init__(self):
        super().__init__('get_object_range')
        def param(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value
        self.base_frame = param('base_frame', 'base_link')
        self.max_age = param('max_sensor_age', 0.5)
        self.future_tolerance = param('max_future_stamp', 0.1)
        self.slop = param('sync_slop', 0.12)
        queue = param('sync_queue_size', 10)
        self.config = FusionConfig(param('roi_fraction', 0.7), param('cluster_gap', 0.15),
                                   param('min_cluster_points', 2))
        if not self.base_frame:
            raise ValueError('base_frame must be nonempty')
        if not all(math.isfinite(v) and v > 0 for v in (self.max_age, self.slop)):
            raise ValueError('max_sensor_age and sync_slop must be positive')
        if not math.isfinite(self.future_tolerance) or not 0 <= self.future_tolerance <= 0.1:
            raise ValueError('max_future_stamp must be in [0, 0.1] seconds')
        if type(queue) is not int or queue < 1:
            raise ValueError('sync_queue_size must be positive')
        self.pub = self.create_publisher(ObjectPolar, param('polar_topic', '/object/polar'), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bearing_sub = message_filters.Subscriber(self, ObjectBearing,
            param('bearing_topic', '/object/bearing'), qos_profile=qos_profile_sensor_data)
        self.bearing_gate = BearingGate()
        # Record loss before the synchronizer considers the same input message.
        self.bearing_sub.registerCallback(self.on_bearing)
        self.scan_sub = message_filters.Subscriber(self, LaserScan,
            param('scan_topic', '/scan'), qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.bearing_sub, self.scan_sub], queue, self.slop)
        self.sync.registerCallback(self.on_pair)

    def invalid(self, stamp):
        msg = ObjectPolar()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.detected = False
        self.pub.publish(msg)

    def on_bearing(self, bearing):
        if not bearing.detected:
            self.bearing_gate.invalidate(stamp_seconds(bearing.header.stamp),
                                        self.get_clock().now().nanoseconds * 1e-9)
            self.invalid(bearing.header.stamp)

    def transform(self, target, source, stamp):
        if target == source:
            return RigidTransform()
        # Do not block the executor waiting for TF: unavailable transforms cause a stop.
        t = self.tf_buffer.lookup_transform(target, source, Time.from_msg(stamp),
                                            timeout=Duration(seconds=0.0)).transform
        return RigidTransform((t.translation.x, t.translation.y, t.translation.z),
                              (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))

    def on_pair(self, bearing, scan):
        camera_time, scan_time = stamp_seconds(bearing.header.stamp), stamp_seconds(scan.header.stamp)
        stamp = bearing.header.stamp if camera_time <= scan_time else scan.header.stamp
        now = self.get_clock().now().nanoseconds * 1e-9
        if (not bearing.detected or not self.bearing_gate.allows(camera_time, now)
                or not bearing.header.frame_id or not scan.header.frame_id
                or not math.isfinite(bearing.confidence) or not 0 < bearing.confidence <= 1
                or abs(camera_time-scan_time) > self.slop
                or min(camera_time, scan_time) <= 0
                or any(now-t > self.max_age or t-now > self.future_tolerance
                       for t in (camera_time, scan_time))):
            self.invalid(stamp)
            return
        try:
            optical = self.transform(bearing.header.frame_id, scan.header.frame_id, scan.header.stamp)
            base = self.transform(self.base_frame, scan.header.frame_id, scan.header.stamp)
            estimate = estimate_range(scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max, bearing.bearing, bearing.angle_min,
                bearing.angle_max, optical, base, self.config)
        except (TransformException, ValueError) as exc:
            self.get_logger().warning(f'Cannot associate target: {exc}', throttle_duration_sec=2.0)
            self.invalid(stamp)
            return
        if estimate is None:
            self.invalid(stamp)
            return
        msg = ObjectPolar()
        msg.header.stamp, msg.header.frame_id = stamp, self.base_frame
        msg.detected = True
        msg.bearing, msg.distance = estimate.bearing, estimate.distance
        msg.confidence = bearing.confidence
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GetObjectRange()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

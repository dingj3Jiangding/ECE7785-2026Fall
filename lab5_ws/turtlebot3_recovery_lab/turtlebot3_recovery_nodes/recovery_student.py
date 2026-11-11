#!/usr/bin/env python3
"""Select configured recovery endpoints; Nav2 executes and confirms the diversion."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from turtlebot3_msgs.srv import Sound
from turtlebot3_recovery_lab.srv import DivertRobot

from turtlebot3_recovery_nodes.mission_config import load_mission
from turtlebot3_recovery_nodes.recovery_logic import choose_recovery_waypoint, grid_from_message


class StudentRecoveryServer(Node):
    def __init__(self):
        super().__init__('student_recovery_server')
        mission_file = self.declare_parameter('mission_file', '').value
        self.mission = load_mission(mission_file, require_recovery=True)
        self.base_frame = self.declare_parameter('robot_base_frame', 'base_footprint').value
        if not isinstance(self.base_frame, str) or not self.base_frame.strip():
            raise ValueError('robot_base_frame must be nonempty')
        self.min_distance = self._positive_parameter('min_recovery_distance', 0.4)
        self.clearance = self._positive_parameter('recovery_clearance', 0.2)
        self.pose_timeout = self._positive_parameter('pose_timeout_sec', 1.0)
        self.costmap_timeout = self._positive_parameter('costmap_timeout_sec', 3.0)
        self.occupied_threshold = self._integer_parameter('occupied_threshold', 50, 1, 100)
        self.costmap_threshold = self._integer_parameter('costmap_occupied_threshold', 100, 1, 100)
        self.enable_sound = self.declare_parameter('enable_sound', True).value
        self.start_sound = self._integer_parameter('start_sound', 3, 1, 5)
        self.success_sound = self._integer_parameter('success_sound', 4, 1, 5)
        if self.start_sound == self.success_sound:
            raise ValueError('start_sound and success_sound must be different')

        self._map = None
        self._costmap = None
        self._costmap_received = None
        self._pending_recovery = None
        self._sound_futures = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        # Volatile also matches transient-local publishers; wait for a fresh update.
        costmap_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._on_costmap, costmap_qos)
        self.sound_client = self.create_client(Sound, '/sound')
        self.srv = self.create_service(DivertRobot, '/divert_robot', self.handle_recovery)
        self.complete_srv = self.create_service(Trigger, '/recovery_complete', self.handle_complete)
        self.get_logger().info(
            f'Recovery server ready: {len(self.mission.recovery_waypoints)} configured points; '
            'waiting for /map, global costmap, and localization TF. '
            'Success sound is triggered only after the recovery FollowPath succeeds.')

    def _positive_parameter(self, name, default):
        value = self.declare_parameter(name, default).value
        if isinstance(value, bool) or not isinstance(value, (float, int)):
            raise ValueError(f'{name} must be a positive number')
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be a positive finite number')
        return float(value)

    def _integer_parameter(self, name, default, lower, upper):
        value = self.declare_parameter(name, default).value
        if isinstance(value, bool) or not isinstance(value, int) or not lower <= value <= upper:
            raise ValueError(f'{name} must be an integer in [{lower}, {upper}]')
        return value

    def _read_grid(self, message, label):
        if message.header.frame_id != self.mission.frame_id:
            self.get_logger().error(
                f'{label} frame {message.header.frame_id!r} does not match '
                f'mission frame {self.mission.frame_id!r}')
            return None
        try:
            return grid_from_message(message)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'Invalid {label}: {exc}')
            return None

    def _on_map(self, message):
        self._map = self._read_grid(message, 'map')

    def _on_costmap(self, message):
        self._costmap = self._read_grid(message, 'costmap')
        self._costmap_received = time.monotonic()

    def beep(self, value):
        """Queue an audible request without blocking the navigation service callback."""
        if not self.enable_sound:
            self.get_logger().info(f'Sound {value} disabled (simulation/testing setting)')
            return
        if not self.sound_client.service_is_ready():
            self.get_logger().warn('Cannot beep: /sound unavailable; audible grading item is unmet')
            return
        # At most four requests outstanding if sound hardware stops responding.
        now = time.monotonic()
        alive = []
        for created, future in self._sound_futures:
            if future.done():
                continue
            if now - created > 5.0:
                self.sound_client.remove_pending_request(future)
                self.get_logger().warn('A previous /sound request timed out')
            else:
                alive.append((created, future))
        self._sound_futures = alive
        if len(alive) >= 4:
            self.get_logger().warn('Too many pending /sound requests; skipping this beep')
            return
        request = Sound.Request()
        request.value = value
        try:
            future = self.sound_client.call_async(request)
            self._sound_futures.append((now, future))
            future.add_done_callback(self.sound_response_callback)
        except Exception as exc:
            self.get_logger().error(f'Cannot request sound {value}: {exc}')

    def sound_response_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('TurtleBot3 acknowledged the buzzer request')
            else:
                self.get_logger().warn(f'TurtleBot3 buzzer failed: {result.message}')
        except Exception as exc:
            self.get_logger().error(f'Sound service call failed: {exc}')

    def handle_recovery(self, request, response):
        del request  # DivertRobot.srv intentionally has an empty request.
        response.success = False
        self._pending_recovery = None
        self.get_logger().warn('Nav2 navigation failed: selecting a recovery waypoint')
        self.beep(self.start_sound)
        if self._map is None or self._costmap is None or self._costmap_received is None:
            self.get_logger().error('Recovery needs both /map and /global_costmap/costmap')
            return response
        if time.monotonic() - self._costmap_received > self.costmap_timeout:
            self.get_logger().error('Global costmap is stale; refusing an unchecked recovery target')
            return response
        try:
            transform = self.tf_buffer.lookup_transform(self.mission.frame_id, self.base_frame, Time())
        except TransformException as exc:
            self.get_logger().error(f'Cannot locate robot for recovery: {exc}')
            return response
        stamp = transform.header.stamp
        age = (self.get_clock().now().nanoseconds - stamp.sec * 1_000_000_000 - stamp.nanosec) / 1e9
        if age < -0.1 or age > self.pose_timeout:
            self.get_logger().error(f'Localization TF timestamp is invalid/stale (age {age:.2f}s)')
            return response
        robot = transform.transform.translation
        point = choose_recovery_waypoint(
            self.mission.recovery_waypoints, robot.x, robot.y, self._map, self._costmap,
            self.min_distance, self.clearance, self.occupied_threshold, self.costmap_threshold)
        if point is None:
            self.get_logger().error(
                'No configured recovery point is clear in both maps and far enough from the robot')
            return response
        pose = response.recovery_pose
        pose.header.frame_id = self.mission.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(point.yaw / 2.0)
        pose.pose.orientation.w = math.cos(point.yaw / 2.0)
        self._pending_recovery = point
        response.success = True
        self.get_logger().info(
            f'Recovery endpoint selected: {point.name} ({point.x:.2f}, {point.y:.2f}); '
            'Nav2 will now check the route and execute it. This is not arrival confirmation.')
        return response

    def handle_complete(self, request, response):
        del request
        point = self._pending_recovery
        if point is None:
            response.success = False
            response.message = 'No pending recovery to complete'
            return response
        # Called by RecoveryComplete in the BT only AFTER FollowPath returns SUCCESS.
        # Do not infer arrival merely from selecting an endpoint or proximity alone.
        self._pending_recovery = None
        self.beep(self.success_sound)
        response.success = True
        response.message = f'Recovery waypoint {point.name} reached; retry original goal'
        self.get_logger().info(response.message)
        # Missing sound is logged separately; it must not strand a recovered robot.
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = StudentRecoveryServer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ValueError, TypeError) as exc:
        print(f'Recovery configuration error: {exc}', flush=True)
        code = 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())

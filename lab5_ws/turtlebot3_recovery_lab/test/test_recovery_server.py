"""ROS-free recovery callback tests; test the real endpoint-selection helper."""

import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch


class FakeTransformException(Exception):
    pass


def load_without_ros():
    """Stub ROS only while importing, then restore sys.modules for other tests."""
    modules = {}
    for name in ('rclpy', 'rclpy.executors', 'rclpy.node', 'rclpy.qos', 'rclpy.time',
                 'nav_msgs', 'nav_msgs.msg', 'std_srvs', 'std_srvs.srv', 'tf2_ros',
                 'turtlebot3_msgs', 'turtlebot3_msgs.srv',
                 'turtlebot3_recovery_lab', 'turtlebot3_recovery_lab.srv',
                 'turtlebot3_recovery_nodes.mission_config'):
        modules[name] = ModuleType(name)
    modules['rclpy.executors'].ExternalShutdownException = type(
        'ExternalShutdownException', (Exception,), {})
    modules['rclpy.node'].Node = type('FakeNode', (), {})
    modules['rclpy.qos'].DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL=1)
    modules['rclpy.qos'].ReliabilityPolicy = SimpleNamespace(RELIABLE=1)
    modules['rclpy.qos'].QoSProfile = object
    modules['rclpy.time'].Time = object
    modules['nav_msgs.msg'].OccupancyGrid = object
    modules['std_srvs.srv'].Trigger = object
    modules['tf2_ros'].Buffer = object
    modules['tf2_ros'].TransformException = FakeTransformException
    modules['tf2_ros'].TransformListener = object
    modules['turtlebot3_msgs.srv'].Sound = object
    modules['turtlebot3_recovery_lab.srv'].DivertRobot = object
    modules['turtlebot3_recovery_nodes.mission_config'].load_mission = object

    source = Path(__file__).resolve().parents[1] / 'turtlebot3_recovery_nodes'
    helper_name = 'turtlebot3_recovery_nodes.recovery_logic'
    helper_spec = importlib.util.spec_from_file_location(
        helper_name, source / 'recovery_logic.py')
    helper = importlib.util.module_from_spec(helper_spec)
    modules[helper_name] = helper
    spec = importlib.util.spec_from_file_location(
        '_recovery_server_under_test', source / 'recovery_student.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        helper_spec.loader.exec_module(helper)
        spec.loader.exec_module(module)
    return module, helper


def recovery_response():
    return SimpleNamespace(
        success=False,
        recovery_pose=SimpleNamespace(
            header=SimpleNamespace(frame_id='', stamp=None),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0))))


class RecoveryServerTests(unittest.TestCase):
    def setUp(self):
        self.module, self.logic = load_without_ros()
        self.module.time = SimpleNamespace(monotonic=lambda: 100.0)
        self.node = self.module.StudentRecoveryServer.__new__(self.module.StudentRecoveryServer)
        self.point = SimpleNamespace(name='safe_corner', x=2.0, y=2.0, yaw=math.pi / 2.0)
        self.node.mission = SimpleNamespace(frame_id='map', recovery_waypoints=(self.point,))
        self.node.base_frame = 'base_footprint'
        self.node.min_distance = 0.4
        self.node.clearance = 0.2
        self.node.pose_timeout = 1.0
        self.node.costmap_timeout = 3.0
        self.node.occupied_threshold = 50
        self.node.costmap_threshold = 100
        self.node.start_sound, self.node.success_sound = 3, 4
        self.node._map = self.logic.Grid(100, 100, 0.1, 0.0, 0.0, 0.0, [0] * 10000)
        self.node._costmap = self.logic.Grid(100, 100, 0.1, 0.0, 0.0, 0.0, [0] * 10000)
        self.node._costmap_received = 100.0
        self.node._pending_recovery = None
        self.transform = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)),
            transform=SimpleNamespace(translation=SimpleNamespace(x=0.5, y=0.5)))
        self.node.tf_buffer = SimpleNamespace(lookup_transform=Mock(return_value=self.transform))
        now = SimpleNamespace(
            nanoseconds=100_000_000_000,
            to_msg=lambda: SimpleNamespace(sec=100, nanosec=0))
        self.node.get_clock = Mock(return_value=SimpleNamespace(now=lambda: now))
        self.logger = Mock()
        self.node.get_logger = Mock(return_value=self.logger)
        self.node.beep = Mock()

    def select(self):
        return self.node.handle_recovery(object(), recovery_response())

    def complete(self):
        return self.node.handle_complete(object(), SimpleNamespace(success=False, message=''))

    def test_selection_beeps_start_and_returns_map_pose_without_success_beep(self):
        response = self.select()
        self.assertTrue(response.success)
        self.node.beep.assert_called_once_with(3)
        pose = response.recovery_pose
        self.assertEqual(pose.header.frame_id, 'map')
        self.assertEqual(pose.header.stamp.sec, 100)
        self.assertEqual((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                         (2.0, 2.0, 0.0))
        self.assertAlmostEqual(pose.pose.orientation.z, math.sqrt(0.5))
        self.assertAlmostEqual(pose.pose.orientation.w, math.sqrt(0.5))
        self.assertEqual((pose.pose.orientation.x, pose.pose.orientation.y), (0.0, 0.0))
        self.assertIs(self.node._pending_recovery, self.point)
        args = self.node.tf_buffer.lookup_transform.call_args.args
        self.assertEqual(args[:2], ('map', 'base_footprint'))

    def test_completion_notification_emits_success_once_and_clears_pending(self):
        self.assertTrue(self.select().success)
        self.node.beep.assert_called_once_with(3)
        response = self.complete()
        self.assertTrue(response.success)
        self.assertIn('safe_corner', response.message)
        self.assertIn('retry original goal', response.message)
        self.assertEqual(self.node.beep.call_args_list, [call(3), call(4)])
        self.assertIsNone(self.node._pending_recovery)
        self.assertFalse(self.complete().success)
        self.assertEqual(self.node.beep.call_count, 2)

    def test_completion_without_pending_recovery_is_rejected(self):
        response = self.complete()
        self.assertFalse(response.success)
        self.assertIn('No pending recovery', response.message)
        self.node.beep.assert_not_called()

    def test_stale_costmap_rejects_recovery_but_still_signals_activation(self):
        self.node._costmap_received = 96.0
        self.assertFalse(self.select().success)
        self.node.beep.assert_called_once_with(3)
        self.node.tf_buffer.lookup_transform.assert_not_called()
        self.assertIsNone(self.node._pending_recovery)

    def test_stale_or_future_tf_rejects_recovery(self):
        for seconds in (98, 101):
            with self.subTest(stamp=seconds):
                self.node.beep.reset_mock()
                self.transform.header.stamp.sec = seconds
                self.assertFalse(self.select().success)
                self.node.beep.assert_called_once_with(3)
                self.assertIsNone(self.node._pending_recovery)

    def test_unavailable_tf_rejects_recovery(self):
        self.node.tf_buffer.lookup_transform.side_effect = FakeTransformException('no map transform')
        self.assertFalse(self.select().success)
        self.node.beep.assert_called_once_with(3)
        self.assertIsNone(self.node._pending_recovery)

    def test_missing_map_or_costmap_rejects_recovery(self):
        for attribute in ('_map', '_costmap', '_costmap_received'):
            with self.subTest(attribute=attribute):
                original = getattr(self.node, attribute)
                setattr(self.node, attribute, None)
                self.assertFalse(self.select().success)
                self.assertIsNone(self.node._pending_recovery)
                setattr(self.node, attribute, original)

    def test_obstructed_endpoint_is_not_reported_as_success(self):
        self.node._costmap.cells[20 * 100 + 20] = 100
        self.assertFalse(self.select().success)
        self.node.beep.assert_called_once_with(3)
        self.assertIsNone(self.node._pending_recovery)
        self.assertFalse(self.complete().success)

    def test_failed_new_selection_clears_previous_pending_recovery(self):
        self.assertTrue(self.select().success)
        self.node._costmap_received = 96.0
        self.assertFalse(self.select().success)
        self.assertFalse(self.complete().success)
        self.assertEqual(self.node.beep.call_args_list, [call(3), call(3)])


if __name__ == '__main__':
    unittest.main()

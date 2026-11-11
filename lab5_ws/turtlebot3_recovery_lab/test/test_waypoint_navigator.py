"""ROS-free action-client tests; these do not replace testing against Nav2."""

import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


class FakeClock:
    def __init__(self):
        self.value = 0.0
        self.interrupt_at = None

    def monotonic(self):
        return self.value

    def spin(self, node, timeout_sec):
        self.value += timeout_sec
        if self.interrupt_at is not None and self.value >= self.interrupt_at:
            self.interrupt_at = None
            raise KeyboardInterrupt()


class FakeFuture:
    def __init__(self, clock, due, value):
        self.clock, self.due, self.value = clock, due, value

    def done(self):
        return self.clock.monotonic() >= self.due

    def result(self):
        if not self.done():
            raise AssertionError('Read a future before it completed')
        if isinstance(self.value, Exception):
            raise self.value
        return self.value


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, text):
        self.messages.append(text)

    warn = info
    error = info


class FakeNode:
    parameters = {}

    def __init__(self, name):
        self.logger = FakeLogger()

    def declare_parameter(self, name, default):
        return SimpleNamespace(value=self.parameters.get(name, default))

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'stamp'))


class FakeGoal:
    def __init__(self):
        self.pose = SimpleNamespace(
            header=SimpleNamespace(frame_id='', stamp=None),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)))
        self.behavior_tree = ''


class FakeHandle:
    def __init__(self, clock, spec):
        self.clock = clock
        self.accepted = spec.get('accepted', True)
        self.cancel_calls = 0
        self.cancel_delay = spec.get('cancel_delay', 0.05)
        self.cancel_acknowledged = spec.get('cancel_acknowledged', True)
        self.result_future = FakeFuture(
            clock, clock.monotonic() + spec.get('duration', 0.2),
            SimpleNamespace(status=spec.get('status', 4)))
        self.result_error = spec.get('result_error', False)

    def get_result_async(self):
        if self.result_error and self.cancel_calls == 0:
            return FakeFuture(self.clock, self.clock.monotonic(), RuntimeError('result transport failed'))
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.cancel_acknowledged:
            self.result_future.due = self.clock.monotonic() + self.cancel_delay
            self.result_future.value = SimpleNamespace(status=5)
        return FakeFuture(self.clock, self.clock.monotonic(), SimpleNamespace(
            goals_canceling=[object()] if self.cancel_acknowledged else []))


class FakeClient:
    def __init__(self, clock, specs):
        self.clock, self.specs = clock, list(specs)
        self.goals, self.handles = [], []

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal, feedback_callback):
        if self.handles:
            previous = self.handles[-1].result_future
            if not previous.done() or previous.result().status != 4:
                raise AssertionError('Sent another goal before previous success')
        spec = self.specs[len(self.goals)]
        handle = FakeHandle(self.clock, spec)
        self.goals.append(goal)
        self.handles.append(handle)
        return FakeFuture(
            self.clock, self.clock.monotonic() + spec.get('acceptance_delay', 0.0), handle)


def load_without_ros():
    """Import only this module with temporary dependency stubs, without pollution."""
    modules = {}
    for name in ('rclpy', 'rclpy.action', 'rclpy.node', 'rclpy.signals',
                 'action_msgs', 'action_msgs.msg', 'nav2_msgs', 'nav2_msgs.action',
                 'turtlebot3_recovery_nodes.mission_config'):
        modules[name] = ModuleType(name)
    modules['rclpy.action'].ActionClient = object
    modules['rclpy.node'].Node = FakeNode
    modules['rclpy.signals'].SignalHandlerOptions = SimpleNamespace(NO=0)
    modules['action_msgs.msg'].GoalStatus = SimpleNamespace(
        STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6)
    modules['nav2_msgs.action'].NavigateToPose = SimpleNamespace(Goal=FakeGoal)
    modules['turtlebot3_recovery_nodes.mission_config'].load_mission = object
    path = Path(__file__).resolve().parents[1] / 'turtlebot3_recovery_nodes/waypoint_navigator.py'
    spec = importlib.util.spec_from_file_location('_waypoint_navigator_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class WaypointNavigatorTests(unittest.TestCase):
    def setUp(self):
        self.module = load_without_ros()
        self.clock = FakeClock()
        self.module.time = SimpleNamespace(monotonic=self.clock.monotonic)
        self.module.rclpy = SimpleNamespace(ok=lambda: True, spin_once=self.clock.spin)
        self.waypoints = tuple(SimpleNamespace(
            name=f'goal_{i}', x=float(i), y=-float(i), yaw=math.pi / 2.0)
            for i in range(1, 4))
        self.module.load_mission = lambda *args, **kwargs: SimpleNamespace(
            frame_id='map', waypoints=self.waypoints)
        FakeNode.parameters = {
            'mission_file': 'test-mission.yaml',
            'behavior_tree': str(Path(__file__).resolve()),
            'server_timeout_sec': 1.0,
            'goal_timeout_sec': 2.0,
            'cancel_timeout_sec': 0.5,
        }

    def node(self, specs):
        self.client = FakeClient(self.clock, specs)
        self.module.ActionClient = lambda *args: self.client
        return self.module.WaypointNavigator()

    def test_three_goals_wait_for_each_success_and_encode_pose(self):
        node = self.node([{}, {}, {}])
        self.assertEqual(node.run(), 0)
        self.assertEqual(len(self.client.goals), 3)
        for goal, waypoint in zip(self.client.goals, self.waypoints):
            self.assertEqual(goal.pose.header.frame_id, 'map')
            self.assertEqual(goal.pose.pose.position.x, waypoint.x)
            self.assertEqual(goal.pose.pose.position.y, waypoint.y)
            self.assertAlmostEqual(goal.pose.pose.orientation.z, math.sqrt(0.5))
            self.assertAlmostEqual(goal.pose.pose.orientation.w, math.sqrt(0.5))
            self.assertEqual(goal.behavior_tree, FakeNode.parameters['behavior_tree'])

    def test_abort_or_canceled_result_stops_mission(self):
        for status in (5, 6):
            with self.subTest(status=status):
                node = self.node([{'status': status}])
                self.assertEqual(node.run(), 1)
                self.assertEqual(len(self.client.goals), 1)

    def test_rejected_goal_does_not_advance_or_cancel(self):
        node = self.node([{'accepted': False}])
        self.assertEqual(node.run(), 1)
        self.assertEqual(len(self.client.goals), 1)
        self.assertEqual(self.client.handles[0].cancel_calls, 0)

    def test_timeout_cancels_and_waits_for_terminal_result(self):
        FakeNode.parameters['goal_timeout_sec'] = 0.3
        node = self.node([{'duration': 20.0, 'cancel_delay': 0.2}])
        self.assertEqual(node.run(), 1)
        handle = self.client.handles[0]
        self.assertEqual(handle.cancel_calls, 1)
        self.assertTrue(handle.result_future.done())
        self.assertEqual(handle.result_future.result().status, 5)
        self.assertEqual(len(self.client.goals), 1)

    def test_late_acceptance_after_send_timeout_is_canceled(self):
        FakeNode.parameters['server_timeout_sec'] = 0.1
        node = self.node([{'acceptance_delay': 0.2, 'duration': 20.0}])
        self.assertEqual(node.run(), 1)
        self.assertEqual(self.client.handles[0].cancel_calls, 1)
        self.assertEqual(self.client.handles[0].result_future.result().status, 5)

    def test_interrupt_cancels_accepted_goal(self):
        self.clock.interrupt_at = 0.1
        node = self.node([{'duration': 20.0}])
        self.assertEqual(node.run(), 130)
        self.assertEqual(self.client.handles[0].cancel_calls, 1)
        self.assertEqual(self.client.handles[0].result_future.result().status, 5)

    def test_unconfirmed_cancel_is_reported_and_never_advances(self):
        FakeNode.parameters['goal_timeout_sec'] = 0.2
        node = self.node([{'duration': 20.0, 'cancel_acknowledged': False}])
        self.assertEqual(node.run(), 1)
        self.assertEqual(len(self.client.goals), 1)
        self.assertTrue(any('Navigation may still be active' in message
                            for message in node.logger.messages))

    def test_failed_result_future_still_cancels_accepted_goal(self):
        node = self.node([{'duration': 20.0, 'result_error': True}])
        self.assertEqual(node.run(), 1)
        handle = self.client.handles[0]
        self.assertEqual(handle.cancel_calls, 1)
        self.assertTrue(handle.result_future.done())
        self.assertEqual(handle.result_future.result().status, 5)
        self.assertEqual(len(self.client.goals), 1)

    def test_nonterminal_result_is_not_proof_goal_stopped(self):
        node = self.node([{'status': 2}])
        self.assertEqual(node.run(), 1)
        self.assertEqual(self.client.handles[0].cancel_calls, 1)
        self.assertEqual(self.client.handles[0].result_future.result().status, 5)
        self.assertEqual(len(self.client.goals), 1)

    def test_unresolved_acceptance_is_reported(self):
        FakeNode.parameters['server_timeout_sec'] = 0.1
        node = self.node([{'acceptance_delay': 20.0, 'duration': 50.0}])
        self.assertEqual(node.run(), 1)
        self.assertTrue(any('Goal acceptance is unresolved' in message
                            for message in node.logger.messages))
        self.assertEqual(len(self.client.goals), 1)

    def test_nonpositive_or_nonfinite_timeout_is_rejected(self):
        for value in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(value=value):
                FakeNode.parameters['goal_timeout_sec'] = value
                with self.assertRaises(ValueError):
                    self.node([])


if __name__ == '__main__':
    unittest.main()

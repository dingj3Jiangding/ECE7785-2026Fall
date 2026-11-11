#!/usr/bin/env python3
"""Send configured map goals one at a time, including Nav2's recovery retries."""

import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from turtlebot3_recovery_nodes.mission_config import load_mission


TERMINAL_STATUSES = (
    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED)


class WaypointNavigator(Node):
    """Keep one NavigateToPose goal active until its final action result."""

    def __init__(self):
        super().__init__('waypoint_navigator')
        mission_file = self.declare_parameter('mission_file', '').value
        self.behavior_tree = self.declare_parameter('behavior_tree', '').value
        self.server_timeout = self._positive_parameter('server_timeout_sec', 60.0)
        self.goal_timeout = self._positive_parameter('goal_timeout_sec', 300.0)
        self.cancel_timeout = self._positive_parameter('cancel_timeout_sec', 5.0)
        if not mission_file:
            raise ValueError('Set mission_file to a YAML file containing your map waypoints.')
        self.mission = load_mission(mission_file, require_waypoints=True)
        if self.behavior_tree and not Path(self.behavior_tree).is_file():
            raise ValueError(f'Behavior tree file does not exist: {self.behavior_tree}')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._pending_goal = None
        self._goal_handle = None
        self._result_future = None
        self._last_feedback_time = -math.inf

    def _positive_parameter(self, name, default):
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be a finite number greater than zero.')
        return value

    def _wait_future(self, future, deadline):
        """Use wall time so a stopped simulation clock cannot stall a timeout."""
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        return future.done()

    def _accept_pending_goal(self):
        handle = self._pending_goal.result()
        self._pending_goal = None
        if handle is None or not handle.accepted:
            return False
        self._goal_handle = handle
        self._result_future = handle.get_result_async()
        return True

    def _clear_goal(self):
        self._goal_handle = None
        self._result_future = None
        self._pending_goal = None

    def _terminal_result_confirmed(self):
        if self._result_future is None or not self._result_future.done():
            return False
        try:
            result = self._result_future.result()
            if getattr(result, 'status', None) in TERMINAL_STATUSES:
                return True
            self.get_logger().warn('Result response did not contain a terminal action status.')
        except Exception as exc:
            self.get_logger().warn(f'Could not read the action result: {exc}')
        # A failed result request is not proof that the accepted goal has stopped.
        # Retry this request after sending cancellation, within the same deadline.
        self._result_future = None
        return False

    def _cancel_active_goal(self):
        """Drain late goal acceptance before canceling; never launch another goal.

        No client can confirm cancellation while the action server is unreachable.
        An unresolved response is therefore reported as a failure requiring the
        operator to stop Nav2/the robot before starting another mission.
        """
        deadline = time.monotonic() + self.cancel_timeout
        try:
            if self._pending_goal is not None:
                if not self._wait_future(self._pending_goal, deadline):
                    self.get_logger().error(
                        'Goal acceptance is unresolved. Stop Nav2/the robot before '
                        'restarting: the server may still accept this goal.')
                    return False
                try:
                    accepted = self._accept_pending_goal()
                except Exception:
                    # get_result_async can fail after acceptance is already known.
                    # Preserve the accepted handle and still request cancellation.
                    if self._goal_handle is None:
                        raise
                    self._result_future = None
                    accepted = True
                if not accepted:
                    self._clear_goal()
                    return True
            if self._goal_handle is None:
                return True
            if self._terminal_result_confirmed():
                self._clear_goal()
                return True

            self.get_logger().warn('Canceling the current navigation goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            if not self._wait_future(cancel_future, deadline):
                self.get_logger().error('Navigation cancellation was not acknowledged.')
            else:
                response = cancel_future.result()
                if not response.goals_canceling:
                    self.get_logger().warn(
                        'Server did not accept cancellation; checking the final result.')

            # A cancel acknowledgment alone does not mean the robot has stopped.
            if self._result_future is None:
                self._result_future = self._goal_handle.get_result_async()
            if (self._wait_future(self._result_future, deadline)
                    and self._terminal_result_confirmed()):
                self._clear_goal()
                return True
        except (Exception, KeyboardInterrupt) as exc:
            self.get_logger().error(f'Could not confirm navigation cancellation: {exc}')
        self.get_logger().error(
            'Navigation may still be active. Stop Nav2/the robot before restarting.')
        return False

    def _feedback(self, message):
        now = time.monotonic()
        if now - self._last_feedback_time < 5.0:
            return
        self._last_feedback_time = now
        feedback = message.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f} m; '
            f'Nav2 recoveries: {feedback.number_of_recoveries}')

    def _navigate(self, waypoint):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.mission.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint.x
        goal.pose.pose.position.y = waypoint.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        goal.behavior_tree = self.behavior_tree
        self._last_feedback_time = -math.inf
        deadline = time.monotonic() + self.goal_timeout
        self._pending_goal = self.client.send_goal_async(
            goal, feedback_callback=self._feedback)
        acceptance_deadline = min(deadline, time.monotonic() + self.server_timeout)
        if not self._wait_future(self._pending_goal, acceptance_deadline):
            self.get_logger().error(f'Timed out waiting for acceptance: {waypoint.name}')
            self._cancel_active_goal()
            return False
        if not self._accept_pending_goal():
            self.get_logger().error(f'Nav2 rejected goal: {waypoint.name}')
            return False
        if not self._wait_future(self._result_future, deadline):
            self.get_logger().error(
                f'Goal exceeded {self.goal_timeout:.1f} s: {waypoint.name}')
            self._cancel_active_goal()
            return False
        result = self._result_future.result()
        if getattr(result, 'status', None) not in TERMINAL_STATUSES:
            raise RuntimeError('Nav2 returned a nonterminal action result; canceling the goal.')
        self._clear_goal()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Navigation failed for {waypoint.name}: action status {result.status}. '
                'The remaining waypoints will not be sent.')
            return False
        self.get_logger().info(f'Reached waypoint: {waypoint.name}')
        return True

    def run(self):
        """Return zero only after every configured waypoint has succeeded."""
        try:
            self.get_logger().info('Waiting for the Nav2 navigate_to_pose action server...')
            deadline = time.monotonic() + self.server_timeout
            while rclpy.ok():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    self.get_logger().error('Nav2 action server did not become available.')
                    return 1
                if self.client.wait_for_server(timeout_sec=min(0.2, remaining)):
                    break
            if not rclpy.ok():
                return 1
            for index, waypoint in enumerate(self.mission.waypoints, start=1):
                self.get_logger().info(
                    f'Navigating to {index}/{len(self.mission.waypoints)}: '
                    f'{waypoint.name} ({waypoint.x:.2f}, {waypoint.y:.2f}), '
                    f'yaw={waypoint.yaw:.2f} rad')
                if not self._navigate(waypoint):
                    return 1
            self.get_logger().info('Mission complete: all waypoints reached.')
            return 0
        except KeyboardInterrupt:
            self.get_logger().warn('Mission interrupted.')
            self._cancel_active_goal()
            return 130
        except Exception as exc:
            self.get_logger().error(f'Mission failed: {exc}')
            self._cancel_active_goal()
            return 1


def main(args=None):
    # Keep the context alive during Ctrl-C so the cancellation handshake can run.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = WaypointNavigator()
        return node.run()
    except (ValueError, OSError) as exc:
        print(f'Waypoint configuration error: {exc}', file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

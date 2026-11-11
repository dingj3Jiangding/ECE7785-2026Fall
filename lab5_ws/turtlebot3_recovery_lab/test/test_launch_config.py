"""Offline checks for Humble bringup configuration and controller consistency."""

from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
PARAMS = yaml.safe_load((PACKAGE / 'config' / 'nav2_params.yaml').read_text())


def _clock_values(value):
    if isinstance(value, dict):
        for key, child in value.items():
            if key == 'use_sim_time':
                yield child
            else:
                yield from _clock_values(child)


def test_default_clocks_are_real_time_everywhere():
    clocks = list(_clock_values(PARAMS))
    assert clocks
    assert all(value is False for value in clocks)


def test_humble_navigation_servers_have_parameters():
    # All these nodes are launched/managed by Humble navigation_launch.py.
    required = {
        'controller_server', 'planner_server', 'smoother_server',
        'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother',
    }
    assert required <= PARAMS.keys()
    assert 'recoveries_server' not in PARAMS
    behavior = PARAMS['behavior_server']['ros__parameters']
    for name in behavior['behavior_plugins']:
        assert behavior[name]['plugin'].startswith('nav2_behaviors/')


def test_controller_and_smoother_velocity_limits_agree():
    controller = PARAMS['controller_server']['ros__parameters']
    dwb = controller['FollowPath']
    smoother = PARAMS['velocity_smoother']['ros__parameters']
    assert 0.0 < dwb['trans_stopped_velocity'] < dwb['max_vel_x']
    assert dwb['max_vel_x'] == dwb['max_speed_xy'] == smoother['max_velocity'][0]
    assert dwb['max_vel_theta'] == smoother['max_velocity'][2]
    assert dwb['xy_goal_tolerance'] == controller['general_goal_checker']['xy_goal_tolerance']


def test_costmaps_have_consistent_robot_size_and_distinct_frames():
    local = PARAMS['local_costmap']['local_costmap']['ros__parameters']
    global_ = PARAMS['global_costmap']['global_costmap']['ros__parameters']
    assert local['robot_radius'] == global_['robot_radius']
    assert local['robot_base_frame'] == global_['robot_base_frame'] == 'base_footprint'
    assert local['global_frame'] == 'odom'
    assert global_['global_frame'] == 'map'
    assert PARAMS['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False


def test_custom_bt_plugin_is_registered_without_unexpanded_xml_path():
    navigator = PARAMS['bt_navigator']['ros__parameters']
    assert 'beep_and_divert_bridge_plugin' in navigator['plugin_lib_names']
    # The launch file provides the installed path rather than relying on YAML expansion.
    assert navigator['default_nav_to_pose_bt_xml'] == ''

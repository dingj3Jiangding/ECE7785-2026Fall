# Lab 4: ROS 2 waypoint navigation and LiDAR obstacle avoidance

[English](README.md) | [中文详细说明](README_中文.md)

Lab partners: **Ding Jiang, Tongning Zhang**

This initial implementation builds on the working Lab 3 robot interfaces: `/scan`, `base_link`, `/cmd_vel` with `Twist` by default, a 20 Hz control loop, and speed limits of 0.15 m/s and 0.8 rad/s. It adds odometry relative to the starting pose, sequential waypoint navigation, local obstacle avoidance, and a 10-second stop at each waypoint. No camera or Lab 3 custom message package is required.

The single ROS 2 node, `navigate_to_goal`, subscribes to `/odom`, `/scan`, and TF. It publishes `/cmd_vel`, `/navigation/pose` (`PoseStamped`), and `/navigation/state` (JSON in `String`). Internal Python modules implement geometry, scan processing, navigation, and sensor watchdogs; they are not separate ROS nodes.

The default route is `(1.5, 0) → (1.5, 1.4) → (0, 1.4)` metres, with arrival tolerances of 0.08, 0.12, and 0.16 m. The first valid robot pose defines the origin and positive x direction, using the same planar transformation as the instructor's rotation script. Restart from the designated starting position. This does not reset `/odom` or eliminate odometry drift.

## Build and run

Use the Ubuntu / ROS 2 installation and robot bringup already validated for Lab 3. This example assumes Humble and `~/lab4_ws`; replace the distribution and path as needed.

```bash
source /opt/ros/humble/setup.bash
cd ~/lab4_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Start the existing base, odometry, LiDAR, and TF drivers. Stop Lab 3 navigation and teleop before launching Lab 4 so only one controller publishes velocity commands. Place the robot at the lab origin, facing positive x.

```bash
ros2 launch team_navigate_to_goal navigate_to_goal.launch.py
```

The launch file locates installed configuration and waypoints independently of the current working directory. To edit source parameters directly:

```bash
ros2 launch team_navigate_to_goal navigate_to_goal.launch.py \
  params_file:="$HOME/lab4_ws/src/team_navigate_to_goal/config/lab4.yaml"
```

Parameters are read-only after startup; restart after edits. Keep `use_sim_time` false on hardware. Set `cmd_vel_stamped: true` only if the base requires `TwistStamped`. Use an absolute path for a custom `waypoints_file`; an empty value selects the installed waypoint file.

Observe each topic in a separate terminal after sourcing ROS 2 and the workspace:

```bash
ros2 topic echo /navigation/pose
ros2 topic echo /navigation/state
ros2 topic echo /cmd_vel
```

## Tune and validate

`robot_radius: 0.22` m is an initial envelope estimate, not a confirmed robot model dimension. Measure the farthest horizontal robot edge from `base_frame`, then tune `safety_margin`, `wall_distance`, and `obstacle_trigger_distance`. Use the real laser-to-base TF. The controller requires a 360-degree horizontal scan and stops for missing or stale sensors, unavailable TF, unknown required laser sectors, or obstacles inside the inflated footprint.

The algorithm follows `GO_TO_GOAL → FOLLOW_WALL → GO_TO_GOAL`, holds each waypoint in `HOLD`, and stops permanently in `DONE` or `BLOCKED`. It is a reactive controller for boxes with room to bypass; it does not guarantee navigation through arbitrary mazes. Check empty-space waypoint motion first, then one box, rotated boxes, and the full lab route. Keep the base's command timeout enabled.

Run ROS-free checks from this workspace directory:

```bash
python3 -m unittest discover -s src/team_navigate_to_goal/test -p 'test_*.py' -v
python3 src/team_navigate_to_goal/test/simulation.py
```

All **23 local unittest tests passed**: 9 navigation tests and 14 sensor/geometry tests. Ideal 2D simulations completed the empty route, boxes at 0° / 30° / 60° / 90°, and a 60° regression with changed arrival tolerances within 150 seconds, with three 10-second stops and no collisions. These results cover synthetic scenarios and do not establish hardware performance.

On the robot, after building:

```bash
colcon test --packages-select team_navigate_to_goal
colcon test-result --verbose
```

For ROS 2 integration checks without a robot, build the workspace, source ROS 2 and `install/setup.bash`, then run both velocity interfaces:

```bash
python3 src/team_navigate_to_goal/test/ros_smoke.py
python3 src/team_navigate_to_goal/test/ros_smoke.py --stamped
```

These standalone checks exercise the actual node with synthetic odometry, scans, and TF, including transforms, nearby-obstacle stops, and separate LiDAR and odometry dropouts. Each run uses UUID-isolated data and TF topics and does not publish to the base's regular `/cmd_vel`. Neither mode has been executed yet; both require a ROS 2 environment and are separate from `colcon test`.

The development environment is macOS without ROS 2. ROS integration and hardware behavior have not yet been verified. The Python simulator uses ideal scans and odometry without slip, noise, or communication delay. See the [Chinese guide](README_中文.md) for the computation graph, frame conventions, parameter definitions, troubleshooting, and first hardware test procedure.

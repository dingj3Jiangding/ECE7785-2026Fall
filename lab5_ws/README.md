# Lab 5: TurtleBot3 navigation and recovery

ROS 2 Humble / physical TurtleBot3 Burger. See [中文运行说明](README_中文.md) for setup, mapping, configuration, launch commands, tests, and grading checks.

The existing package is intentionally kept directly under `lab5_ws`:

```bash
source /opt/ros/humble/setup.bash
# Run from lab5_ws; source your TurtleBot3 overlay too if installed from source.
rosdep install --from-paths turtlebot3_recovery_lab --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --base-paths turtlebot3_recovery_lab
source install/setup.bash
```

Fill `turtlebot3_recovery_lab/config/mission.yaml` with measured map-frame goals and at least one tested recovery location before launch. The shipped lists are empty on purpose. Save your map YAML and its referenced image together.

```bash
# Terminal A: robot drivers must already be running.
ros2 launch turtlebot3_recovery_lab lab5_bringup.launch.py \
  map:=/absolute/path/to/maze.yaml \
  mission_file:=/absolute/path/to/mission.yaml use_sim_time:=False

# Initialize AMCL in RViz and confirm localization before Terminal B:
ros2 launch turtlebot3_recovery_lab waypoints.launch.py \
  mission_file:=/absolute/path/to/mission.yaml use_sim_time:=False
```

The client waits for each NavigateToPose action's final success before sending the next. Recovery uses validated configured endpoints, Nav2 planning and tracking, an activation beep, a distinct completion beep only after the recovery FollowPath succeeds, and retries of the original goal. Endpoint validation does not guarantee that a full path exists. Failure stops the route rather than skipping a required goal.

The current machine has no ROS installation. Offline logic/configuration tests passed; ROS compilation, middleware integration and hardware behavior still need to be checked on the Humble workstation. Default speed, clearance and tolerances are initial settings, not hardware-validated tuning.

```bash
colcon test --packages-select turtlebot3_recovery_lab --event-handlers console_direct+
colcon test-result --verbose
```

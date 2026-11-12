# Lab 5: TurtleBot3 navigation and recovery

ROS 2 Humble / physical TurtleBot3 Burger. See [中文运行说明](README_中文.md) for setup, mapping, configuration, launch commands, tests, and grading checks.

## Run with the helper script

`run_lab5.sh` wraps the source/build/launch sequence used in Lab 4. On the Ubuntu / ROS 2 Humble workstation, install the dependencies described below, fill the mission file, and save the map before running:

```bash
# From lab5_ws; build once and repeat after code changes.
./run_lab5.sh build

# Terminal A: existing robot drivers must already be running.
./run_lab5.sh bringup

# Terminal B: after setting the initial pose in RViz and checking localization.
./run_lab5.sh waypoints
```

The script sources ROS and, for launch commands, this workspace's `install/setup.bash`. It finds the workspace from its own location, so an absolute path to the script also works from another directory. `bringup` starts Nav2 and recovery only; `waypoints` starts robot navigation.

Defaults are `maps/maze.yaml`, the source `config/mission.yaml`, the source `config/nav2_params.yaml`, and `use_sim_time:=False`. Map and mission coordinates must still be supplied. Normal ROS launch arguments override defaults:

```bash
./run_lab5.sh bringup map:="/absolute/path/to/my map.yaml" rviz:=False
./run_lab5.sh waypoints goal_timeout_sec:=600.0
./run_lab5.sh --help
```

Use the same `mission_file:=...` and `use_sim_time:=...` in both terminals when overriding them. Extra arguments after `build` are forwarded to `colcon build`.

The environment defaults match the repository's `do_config.sh`: `TURTLEBOT3_MODEL=burger`, `ROS_DOMAIN_ID=66`, and `LDS_MODEL=LDS-02`; existing environment values take precedence. Make the domain ID match the robot in both terminals. The script loads `/opt/ros/humble/setup.bash`, then `~/turtlebot3_ws/install/setup.bash` if present. Set `LAB5_ROS_SETUP` or `LAB5_TURTLEBOT3_SETUP` to select other setup files. Running the script does not source these environments into the calling terminal; use the manual commands below for separate diagnostic commands.

## Manual build and launch

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

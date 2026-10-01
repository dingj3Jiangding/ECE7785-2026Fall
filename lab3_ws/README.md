# Lab 3: ROS 2 Colored Object Following

[English](README.md) | [中文](README_中文.md)

Student 1: **Ding Jiang**  
Student 2: **Tongning Zhang**

This project targets TurtleBot3 with ROS 2 Humble or Jazzy on Ubuntu. It includes lab code and launch configuration. By default, it tracks a green target: the camera estimates its direction, LiDAR estimates its range, and the robot follows it. The package name `team_chase_object` uses `team` as a placeholder team name and satisfies the lowercase naming requirement. Before submission, replace both student names here and in the source file headers. If you change the team package name, also update the package directory, package references other than message imports, metadata, and launch configuration.

The default configuration must be calibrated for the actual robot. The current development environment is macOS without ROS 2. Local checks cannot replace an Ubuntu build, sensor integration, or a live robot demonstration. No experimental results have been filled in or fabricated for this project.

## 1. Files and data flow

```text
lab3_ws/
├── src/
│   ├── chase_object_interfaces/   # Custom ROS 2 messages
│   └── team_chase_object/
│       ├── team_chase_object/     # Detection, ranging, and control nodes
│       ├── config/lab3.yaml       # Topics, color, TF, PID, and speed limits
│       └── launch/chase_object.launch.py
├── README.md
└── README_中文.md
```

| Node | Inputs | Output and responsibility |
|---|---|---|
| `detect_object` | Compressed camera image and camera intrinsics | `/object/bearing` (`ObjectBearing`); segments the target color and estimates its image bearing |
| `get_object_range` | Target bearing, `/scan`, and TF | `/object/polar` (`ObjectPolar`); associates laser returns with the visual target and computes its range and bearing |
| `chase_object` | Target range and bearing | `/cmd_vel`; follows the target with separate distance and heading control loops |

The default camera image topic is `/camera/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`), and the camera intrinsics topic is `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`). The default laser topic is `/scan` (`sensor_msgs/msg/LaserScan`). The robot base may accept `Twist` or `TwistStamped`; by default, `cmd_vel_stamped: false` publishes `Twist`. Set it to `true` if the base subscribes to `TwistStamped`.

The suggested default is to run all three nodes on the robot, consistent with the existing computation graph. The launch file starts only this lab's three nodes. Existing drivers must still start the base, camera, LiDAR, and robot TF.

Parameters are read at startup and declared read-only. Restart the nodes after editing the YAML file; `ros2 param set` cannot change these settings while they run.

## 2. Build in a ROS 2 workspace

The following assumes ROS 2, `colcon`, and `rosdep` are installed. Humble corresponds to Ubuntu 22.04; Jazzy corresponds to Ubuntu 24.04. Copy both packages in this directory's `src` folder to a ROS 2 workspace on the robot, such as `~/lab3_ws/src/`. If you copy the whole `lab3_ws` directory to Ubuntu, you can build there directly.

```bash
# Humble example; replace humble with jazzy when using Jazzy.
source /opt/ros/humble/setup.bash
cd ~/lab3_ws

# If rosdep has not been initialized, run sudo rosdep init first.
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

ros2 interface show chase_object_interfaces/msg/ObjectBearing
ros2 interface show chase_object_interfaces/msg/ObjectPolar
```

Source ROS 2 and this workspace's `install/setup.bash` in every new terminal. If sensors run on another computer, configure ROS 2 discovery and `ROS_DOMAIN_ID` correctly on both machines.

## 3. Check sensors and coordinate frames first

Start the base, camera, LiDAR, and TF using the robot's existing setup, then run:

```bash
ros2 topic list -t
ros2 topic info /camera/image_raw/compressed --verbose
ros2 topic echo /camera/camera_info --once
ros2 topic echo /scan --once
ros2 topic info /cmd_vel --verbose
```

Update `src/team_chase_object/config/lab3.yaml` with the actual topic names. Use `image_topic` and `camera_info_topic` for the camera. `image_transport` must match the message type: the default `compressed` uses `CompressedImage`; use `raw` with a regular `Image`. Check the `header.frame_id` values in the output above for the camera and laser frame names.

TF must connect three real frames: the laser scan frame, the camera optical frame, and the robot's `base_link` (or the configured base frame). Camera optical axes point right, down, and forward; typical base-frame axes point forward, left, and up. The camera and LiDAR also have a physical offset and relative rotation. Use actual extrinsics from calibration or the robot URDF. An incorrect identity transform will produce wrong object associations and turning directions.

Check TF connectivity with these commands, replacing the placeholders with actual frame names:

```bash
ros2 run tf2_ros tf2_echo base_link ACTUAL_LASER_FRAME
ros2 run tf2_ros tf2_echo ACTUAL_CAMERA_OPTICAL_FRAME ACTUAL_LASER_FRAME
```

The camera should preferably publish calibrated `CameraInfo`. Without usable intrinsics, the code approximates bearing using the configured horizontal field of view. Enter the actual lens's horizontal field of view and verify the bearing for a target on the left, center, and right of the image. The target must intersect the LiDAR's horizontal scan plane; otherwise, the camera can see it but the LiDAR cannot measure its true range. Use a target with a distinct color, sufficient size, and a surface the LiDAR can reliably detect, and avoid backgrounds of the same color.

The ranging node uses TF to project laser points into the camera optical frame, keeps candidate points in the central portion of the visual target's angular interval, and selects a nearby, range-continuous group. By default, it requires at least two valid laser points. A narrow, distant, or vertically misaligned target may fail this filter. Output range and bearing are expressed in the configured base frame, `base_link` by default.

Association uses only horizontal image bearing and assumes the target intersects the scan plane. A closer occluder in the same direction may be mistaken for the target. Use a sufficiently large, opaque target that the LiDAR detects consistently, and keep such occluders out of the demo area. By default, range is the planar distance from the base-frame origin to the visible target surface, not to its geometric center or the robot's bumper.

## 4. Launch and observe

After checking topics, TF, and the base command interface, launch the nodes:

```bash
source /opt/ros/humble/setup.bash
source ~/lab3_ws/install/setup.bash
ros2 launch team_chase_object chase_object.launch.py
```

To use a custom parameter file:

```bash
ros2 launch team_chase_object chase_object.launch.py \
  params_file:="$HOME/lab3_ws/src/team_chase_object/config/lab3.yaml"
```

In simulation, set `use_sim_time:=true` only if the system publishes `/clock`. Keep the default `false` on a real robot.

Observe the complete pipeline in another terminal:

```bash
ros2 topic echo /object/bearing
ros2 topic echo /object/polar
ros2 topic echo /cmd_vel
```

You can also start each node separately to isolate a problem:

```bash
ros2 run team_chase_object detect_object --ros-args \
  --params-file "$HOME/lab3_ws/src/team_chase_object/config/lab3.yaml"
ros2 run team_chase_object get_object_range --ros-args \
  --params-file "$HOME/lab3_ws/src/team_chase_object/config/lab3.yaml"
ros2 run team_chase_object chase_object --ros-args \
  --params-file "$HOME/lab3_ws/src/team_chase_object/config/lab3.yaml"
```

Use a separate terminal for each command. Do not run these alongside the full launch, or multiple nodes may publish velocity commands.

## 5. Tune color, range, and control

1. **Color segmentation:** Adjust `hsv_lower`, `hsv_upper`, `min_area`, and `morph_kernel` under the actual lab lighting. OpenCV's H range is 0–179; S and V range from 0–255. The default green range is only a starting point. Verify color detection with live images and remove distracting objects of the same color.
2. **Bearing and ranging:** Inspect detection and ranging outputs first. Place the target in front of the robot and on both sides; check that bearing signs and ranges match reality. Move it slowly to check whether laser association stays stable. If the target disappears or no laser points match, inspect target height, TF, camera intrinsics, and timestamp synchronization.
3. **Control parameters:** Start with two proportional loops, one for linear velocity and one for angular velocity. Integral and derivative gains remain configurable; begin with `I=0, D=0`. Tune turning at a low speed limit before tuning distance control. Reduce the relevant gain if the robot oscillates, and add derivative gain cautiously if needed. If an integral term is needed for persistent steady-state error, check output limits and anti-windup behavior.
4. **Desired range and stopping error:** Set the desired following distance and allowable error to meet the lab requirements. Check the robot's behavior when the target is too close before increasing speed. Do not assume the defaults are suitable for the real robot.

Main control parameters are listed below. Distances are in meters and angles are in radians.

| Parameter | Default | Meaning |
|---|---:|---|
| `target_distance` | 0.60 | Desired target range |
| `distance_deadband` / `bearing_deadband` | 0.05 / 0.04 | Distance and bearing deadbands |
| `linear_kp` / `linear_ki` / `linear_kd` | 0.5 / 0 / 0 | Distance PID gains |
| `angular_kp` / `angular_ki` / `angular_kd` | 1.5 / 0 / 0 | Heading PID gains |
| `max_forward_speed` / `max_reverse_speed` | 0.15 / 0.10 | Forward and reverse speed limits, m/s |
| `max_angular_speed` | 0.8 | Angular speed limit, rad/s |
| `heading_gate` | 0.6 | Turn first when heading error is large |
| `control_rate_hz` | 20 | Command publication rate |
| `observation_timeout` | 0.5 | Target-data timeout, seconds |
| `min_confidence` | 0.2 | Minimum target confidence |

Here, `confidence` is the color contour area divided by its bounding-box area. It is a detection-quality heuristic, not a statistically calibrated probability. Use `rqt_image_view` to inspect `/object/debug_image` (install the `rqt_image_view` package for your ROS distribution if necessary). The debug image is generated only while a subscriber is present.

The controller stops on invalid targets, timed-out input, or nonfinite values, and its speeds are capped by configuration. During normal shutdown, the node attempts to send a zero-velocity command. A forced process termination or network failure may prevent that message from reaching the base, so configure the base's command-timeout stop on a real robot. This project does not implement obstacle avoidance. For the first test, drive slowly in an open area with someone ready to stop the robot immediately.

## 6. Verification and experimental records

After building on Ubuntu, run:

```bash
cd ~/lab3_ws
colcon test --packages-select team_chase_object
colcon test-result --verbose
```

### ROS 2 smoke test on Ubuntu (no robot required)

After building and sourcing the workspace, run the synthetic end-to-end test:

```bash
source /opt/ros/humble/setup.bash  # Use jazzy for ROS 2 Jazzy.
cd ~/lab3_ws
colcon build --symlink-install
source install/setup.bash
python3 src/team_chase_object/test/ros_smoke.py
```

The script starts all three nodes in one process, publishes a synthetic green image, camera intrinsics, LiDAR scan, and TF, then checks the bearing, range, forward command, sensor-dropout stop, target reacquisition, and lost-target stop. It remaps sensor, TF, and velocity topics into a unique namespace so it does not publish to the robot's normal `/cmd_vel`. Run this script separately; it is not part of `colcon test`. It does not validate hardware calibration, real sensing, or the 5-second demo requirement.

To test only the ROS-independent algorithms, use a Python environment with NumPy, OpenCV, and pytest:

```bash
cd ~/lab3_ws
PYTHONPATH=src/team_chase_object python3 -m pytest -q src/team_chase_object/test
```

A prior macOS check using Python 3.12, OpenCV 5.0.0, and NumPy 2.5.3 reported **46 passing tests**. These cover image compression and segmentation, distortion and bearing conversion, transforms with translation, invalid laser ranges/background/scan boundaries, both PID control loops, timeout and replay protection, and an end-to-end synthetic image-to-ideal-robot pipeline. This result does not mean ROS nodes have run or a real robot has passed the demo.

Fill in the course writeup only after real-robot validation. Suggested scenarios and measurements:

| Scenario | Check and record |
|---|---|
| Target left, center, and right in the image | Stable detection, correct bearing sign, LiDAR range for the same target |
| Stationary target at different locations | Convergence time, final distance and bearing errors, oscillation |
| Slowly moving target | Following stability and compliance with speed limits |
| Target leaves the image or is occluded | Timely zero velocity and normal behavior when detection resumes |
| Camera, laser, or target messages stop | Stop after timeout; no motion driven by stale targets |
| Lighting or background changes | False detections, target losses, and recovery |

The manual's **5-second target-state requirement** must be measured on a real robot and tuned using actual results. Default parameters cannot guarantee this for every robot, starting position, and scene. Videos, screenshots, error curves, and timing data must come from the experiment, rather than example or expected behavior presented as measurements.

Before submission, check both student names, whether the team package name meets the course requirement, all source code and message definitions, launch and parameter files, the manual's required written answers, and real demo material.

## 7. Equations and timing conventions

For an undistorted pinhole camera with target-center pixel `u`, horizontal focal length `fx`, and principal point `cx`:

```text
camera_bearing = atan2(cx - u, fx)          # Positive to the left, in rad
laser_angle_i = angle_min + i * angle_increment
p_laser = [r_i*cos(laser_angle_i), r_i*sin(laser_angle_i), 0]
p_camera = R_camera_laser * p_laser + t_camera_laser
point_bearing = atan2(-p_camera.x, p_camera.z)
```

When distortion coefficients are available, pixels are undistorted before calculating bearing. Keep only laser points in front of the camera and within the central `roi_fraction` of the target's horizontal angular interval. Group adjacent points by spatial separation, discard undersized groups, and choose the nearest continuous group. Transform its points to the base frame and take the median of the x and y coordinates separately to estimate the target position:

```text
distance = hypot(x_base, y_base)
bearing = atan2(y_base, x_base)
e_distance = distance - target_distance
e_bearing = bearing
v = clip(PID_distance(e_distance), -max_reverse_speed, max_forward_speed)
w = clip(PID_bearing(e_bearing), -max_angular_speed, max_angular_speed)
```

Inside a corresponding deadband, that control output is set exactly to zero and its integral and derivative state is cleared. When heading error reaches `heading_gate`, the robot turns first and pauses translation. PID integrates and differentiates using the control timer's actual steady-clock interval; I and D gains default to zero. Conditional integration prevents windup, and stopping or losing the target resets the controllers.

Camera and laser observations are approximately synchronized by their source timestamps; the default maximum difference is 0.12 seconds. The output retains the earlier of the two source timestamps so processing time cannot hide stale data. After an explicit target loss, older positive detections still in the queue cannot restart the robot. The controller checks both source timestamps and elapsed steady-clock time since receipt, rejecting duplicate or out-of-order timestamps. A new timestamp sequence starts only after an actual ROS clock reset.

`20 Hz` is the command publication rate, not necessarily the effective sensor update rate. Measure the sampling time for the report using actual update intervals from the camera, `/scan`, and `/object/polar`. You can use `ros2 topic hz`; also record jitter and end-to-end latency. When nodes run on different computers, synchronize their system clocks or the timestamp checks may stop the robot.

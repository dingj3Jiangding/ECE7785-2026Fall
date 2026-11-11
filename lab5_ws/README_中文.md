# Lab 5：TurtleBot3 Burger 自主导航与恢复

适用环境：Ubuntu 22.04、ROS 2 Humble、实机 TurtleBot3 Burger。保留已有目录结构：`turtlebot3_recovery_lab` 直接放在 `lab5_ws` 下，构建命令显式指定包路径。

## 已实现的行为

1. `waypoint_navigator` 从任务 YAML 读取目标，逐个发送 `NavigateToPose` action。只有前一个目标最终返回 `SUCCEEDED`，才发送下一个；目标被拒绝、失败或取消时停止。
2. 正常规划或跟踪失败时，行为树清空局部 costmap，调用 `/divert_robot`。Python 服务发出启动蜂鸣，从预设恢复点中选择离机器人最近且通过检查的点。
3. 选点检查覆盖静态地图、最新全局 costmap、机器人定位 TF、未知区域、地图边界和机器人周围的空余空间。当前位置附近的点不会被当作有效恢复点。
4. 行为树使用单独的 `recovery_pose` 和 `recovery_path` 规划、跟踪恢复路线，保留原始 `goal`。
5. 恢复路线的 `FollowPath` 返回成功后，`RecoveryComplete` 才调用 `/recovery_complete`。Python 请求不同的成功蜂鸣，随后行为树重试原目标，最多执行 3 次恢复。

`DivertRobot.response.success` 表示选出了通过端点检查的恢复点，并不表示已经到达。路径是否可达由 Nav2 的 `ComputePathToPose` 验证。如果恢复规划或跟踪失败，本次导航失败，脚本停止；不会把失败记为成功，也不会自动切换到另一个全局目标。

## 运行前需要填写的内容

在 `turtlebot3_recovery_lab/config/mission.yaml` 中填写：

- `frame_id`：通常为 `map`，须与保存地图及定位坐标系一致。
- `waypoints`：验收时给出的 3 个点，顺序就是访问顺序。脚本也允许先填一个点做调试。
- `recovery_waypoints`：至少一个事先验证过、留有转向空间且能从测试区域到达的恢复点。
- 每个点使用 `{name, x, y, yaw}`；`x/y` 单位是米，`yaw` 是弧度，省略 yaw 则默认朝向地图的 +x 方向。名字可以省略。

配置模板默认是空列表，启动时会提示填写。没有假定地图原点安全，也没有预装可能误用的示例坐标。可以在建图时通过 RViz 的 Publish Point 和 `ros2 topic echo /clicked_point` 获取地图坐标。目标点的朝向需要另行选择。

恢复点检查默认使用半径 0.20 m 的圆形空余区域。静态地图 occupancy >= 50 视为障碍；全局 costmap 的 lethal 值 100 视为障碍，较低的 inflation 值不再次作为实体障碍膨胀。所有未知单元格都被拒绝。端点检查不证明整条路径畅通，请实际验证恢复位置和进出路线。

## 构建

以下命令在运行 ROS 2 Humble 的 Ubuntu 电脑上执行。先进入该电脑上的 `lab5_ws`，不要直接在当前 macOS 环境尝试运行 ROS 节点。

```bash
source /opt/ros/humble/setup.bash
# 如果按讲义从源码构建过 TurtleBot3，另行 source 对应 workspace 的 install/setup.bash。
export TURTLEBOT3_MODEL=burger
LAB5_WS="$PWD"
rosdep install --from-paths turtlebot3_recovery_lab --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --base-paths turtlebot3_recovery_lab
source install/setup.bash
ros2 pkg executables turtlebot3_recovery_lab
```

最后应能看到 `recovery_server` 和 `waypoint_navigator`。`rosdep` 需要预先完成系统的一次性初始化。代码、C++ 插件或安装资源修改后重新构建，并在各终端重新 source。

## 建图与准备

按照课程 Lab 5 的 ROS 2 Humble SLAM 步骤，用遥控完成地图。真实 Burger 的底盘、LiDAR、里程计和 TF 由已有的 `turtlebot3_bringup` 提供，此包不重复启动机器人驱动。

```bash
mkdir -p "$LAB5_WS/maps"
ros2 run nav2_map_server map_saver_cli -f "$LAB5_WS/maps/maze"
```

保存后应有 `maze.yaml` 及它引用的地图图像；两者一起保存、一起提交。随后停止 SLAM 和遥控速度发布，再进入基于已保存地图的 Nav2 定位导航。所有电脑与机器人保持一致的 ROS_DOMAIN_ID、网络配置和时钟。

## 启动 Nav2 与恢复服务

终端 A：进入 `lab5_ws` 并 source 环境，然后执行：

```bash
LAB5_WS="$PWD"
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_recovery_lab lab5_bringup.launch.py \
  map:="$LAB5_WS/maps/maze.yaml" \
  mission_file:="$LAB5_WS/turtlebot3_recovery_lab/config/mission.yaml" \
  use_sim_time:=False
```

此命令启动 Nav2、恢复服务和 RViz，不会自动发送目标。地图、任务文件会先被检查。只运行这一套 Nav2，不要同时再运行另一套 TurtleBot3 navigation launch。

在 RViz 使用 **2D Pose Estimate** 设置机器人初始位置和朝向，确认 LiDAR 与地图对齐。可先用 RViz 测试单个导航目标；自主脚本运行期间不要再从 RViz 或另一个脚本发送新目标。初始化定位与验收时手动逐个发送目标是不同的操作。

必要检查：

```bash
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic hz /global_costmap/costmap
ros2 service list -t
```

应看到 Nav2 active、有效定位 TF、持续发布的 costmap，以及 `/divert_robot`、`/recovery_complete`、实机的 `/sound` 服务。恢复服务默认要求定位 TF 在 1 s 内、全局 costmap 在最近 3 s 内更新。

## 启动连续目标导航

终端 B：进入同一个 `lab5_ws`，source 相同环境，执行：

```bash
LAB5_WS="$PWD"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch turtlebot3_recovery_lab waypoints.launch.py \
  mission_file:="$LAB5_WS/turtlebot3_recovery_lab/config/mission.yaml" \
  use_sim_time:=False \
  goal_timeout_sec:=300.0
```

两个 launch 使用同一份任务文件和相同的 `use_sim_time`。文件在节点启动时读取；修改恢复点后重启终端 A 的 launch，修改目标列表后重新启动终端 B。代码不会在运动中热加载坐标。

每个目标的 300 s 超时包含所有恢复和重试时间。Ctrl-C 会请求取消当前 action 并等待终态；如果日志明确提示未能确认取消，先停止 Nav2/机器人，确认停止后再启动下一次任务。不要同时运行多个 waypoint 客户端。

直接运行节点也可用于脚本调用（退出码：0 全部成功，1 导航失败，2 配置错误，130 用户中断）：

```bash
ros2 run turtlebot3_recovery_lab waypoint_navigator --ros-args \
  -p mission_file:="$LAB5_WS/turtlebot3_recovery_lab/config/mission.yaml"
```

这一方式使用 Nav2 已加载的默认行为树，因此仍需先通过本包的 bringup 启动 Nav2。

## 参数调整与验收

当前 Burger 初始参数：最大前进速度 0.15 m/s，costmap 机器人半径 0.12 m，目标位置容差 0.15 m，朝向容差 0.25 rad。它们是待实机验证的起点。Nav2 参数位于 `config/nav2_params.yaml`；若要直接使用编辑后的源文件，可给 bringup 增加 `params_file:="$LAB5_WS/turtlebot3_recovery_lab/config/nav2_params.yaml"`。

bringup 还支持 `recovery_clearance:=0.20`、`min_recovery_distance:=0.40`、`pose_timeout_sec:=1.0`、`costmap_timeout_sec:=3.0`、`rviz:=False`。机器人尺寸、恢复点净空和地图分辨率应一起核对。服务端还有 `occupied_threshold`、`costmap_occupied_threshold`、`start_sound`、`success_sound` ROS 参数，默认蜂鸣编号分别为 3 和 4。

验收前依次检查：展示保存的地图；单点到达；三点按顺序到达；人为阻塞原路线后出现启动蜂鸣；机器人驶到选定恢复点后才出现成功蜂鸣；恢复后重试原目标；最终抵达后才进入下一目标。若原路线仍存在其他有效路径，Nav2 可能直接绕行，正常导航成功时不会触发恢复。

若 `/sound` 不可用或响应失败，程序会明确记录告警，不会把蜂鸣记为已成功，也不会因此阻止已经完成的恢复继续重试。实机验收仍必须确认两次蜂鸣都能听见。Gazebo 调试可把两个 launch 的 `use_sim_time` 都设为 `True`，并给 bringup 设置 `enable_sound:=False`；这不能代替实机蜂鸣验收。

## 测试与交付

本机 40 项 ROS-free 测试全部通过，覆盖任务配置验证、三点顺序、失败/取消/超时、恢复端点障碍检查、旋转地图、定位/地图过期和蜂鸣时机；检查了 XML、YAML、Python 语法及 Humble 官方接口。当前 macOS 没有 ROS 2，因此未执行 `colcon build`、真实 DDS 通信、Gazebo 或 Burger 演示，参数尚未经过实机调优。

在 ROS 工作站构建后运行：

```bash
colcon test --packages-select turtlebot3_recovery_lab --event-handlers console_direct+
colcon test-result --verbose
```

也可离线运行（需安装 pytest 和 PyYAML）：

```bash
PYTHONPATH="$LAB5_WS/turtlebot3_recovery_lab${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m pytest -q turtlebot3_recovery_lab/test
```

代码通过检查后，仍需完成上面的实机验收。提交 ZIP 应包含 ROS 包、填写后的任务配置、实际地图和使用说明，不包含 `build/`、`install/`、`log/`。按讲义命名为 `TeamName_Lab5.zip`，并完成现场演示。

## 代码位置

- `turtlebot3_recovery_nodes/waypoint_navigator.py`：目标发送、反馈、超时与取消。
- `turtlebot3_recovery_nodes/recovery_student.py`：ROS 恢复服务、地图与 TF 输入、蜂鸣。
- `turtlebot3_recovery_nodes/recovery_logic.py`：可独立测试的地图碰撞与恢复点选择。
- `turtlebot3_recovery_nodes/mission_config.py`：公共任务文件解析。
- `src/beep_and_divert_bridge.cpp`、`include/.../beep_and_divert_bridge.hpp`：保留原桥接，新增实际到达后的完成通知节点。
- `behavior_trees/navigate_w_recovery_divert.xml`：恢复、成功提示与重试的顺序。
- `launch/`、`config/`、`test/`：启动、参数和测试。

新增 C++ 完成通知和 XML 步骤是为了落实讲义“到达恢复点后成功蜂鸣”的要求。原模板 Python 服务只负责返回坐标，无法单靠返回坐标判断已到达。

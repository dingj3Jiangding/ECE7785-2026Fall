# Lab 3：ROS 2 彩色物体跟随

[English](README.md) | [中文](README_中文.md)

学生 1：**Ding Jiang**  
学生 2：**Tongning Zhang**

面向 Ubuntu 上的 ROS 2 Humble / Jazzy 和 TurtleBot3。本项目包含实验代码与启动配置；默认寻找蓝色目标，通过相机确定方向、激光雷达估计距离，再控制机器人跟随。包名 `team_chase_object` 使用 `team` 作为团队名，符合小写命名要求。提交前请替换代码头部和此处的两位学生姓名；如需改团队包名，也要同步修改包目录、消息导入以外的包引用、元数据和启动配置。

代码的默认配置需要按实际机器人校准。当前开发环境是 macOS，没有 ROS 2 运行环境；本地检查不能代替 Ubuntu 上的构建、传感器联调和实机演示。本项目没有填写或虚构实验结果。

## 1. 文件与数据流

```text
lab3_ws/
├── src/
│   ├── chase_object_interfaces/   # 自定义 ROS 2 消息
│   └── team_chase_object/
│       ├── team_chase_object/     # 检测、测距、控制节点
│       ├── config/lab3.yaml       # 话题、颜色、TF、PID 和限速配置
│       └── launch/chase_object.launch.py
├── README.md
└── README_中文.md
```

| 节点 | 输入 | 输出与职责 |
|---|---|---|
| `detect_object` | 压缩相机图像、相机内参 | `/object/bearing`，`ObjectBearing`；分割目标颜色并估计图像中的方位 |
| `get_object_range` | 目标方位、`/scan`、TF | `/object/polar`，`ObjectPolar`；将激光点与视觉目标关联并计算目标的距离和方向 |
| `chase_object` | 目标距离与方向 | `/cmd_vel`；用距离和转向两个控制回路跟随目标 |
| `camera_static_tf` | 启动文件中的相机安装参数 | `/tf_static`；连接 `base_scan` 与相机光学坐标系 `camera` |

随附的 `lab3.yaml` 使用实验机器人实际发布的相机话题：图像话题为 `/image_raw/compressed`，类型为 `sensor_msgs/msg/CompressedImage`；相机内参话题为 `/camera_info`，类型为 `sensor_msgs/msg/CameraInfo`。默认激光话题为 `/scan`，类型为 `sensor_msgs/msg/LaserScan`。机器人底盘可能使用 `Twist` 或 `TwistStamped`；默认 `cmd_vel_stamped: false` 发布 `Twist`，如果底盘订阅 `TwistStamped`，则改为 `true`。

默认建议三个实验节点都在机器人端运行，与已有计算图的部署方式一致。启动文件还会启动 `camera_static_tf`；底盘、相机、雷达以及底盘到雷达的 TF 仍由已有驱动启动。

参数在节点启动时读取并设为只读。修改 YAML 后需要重新启动节点；运行时不能用 `ros2 param set` 改变这些配置。

## 2. 在 ROS 2 工作空间构建

下例假设系统已安装 ROS 2、`colcon` 与 `rosdep`。Humble 对应 Ubuntu 22.04，Jazzy 对应 Ubuntu 24.04。先将本目录中的两个 `src` 包复制到机器人的 ROS 2 工作空间，例如 `~/lab3_ws/src/`。如果直接把整个 `lab3_ws` 目录复制到 Ubuntu，也可在该目录构建。

```bash
# Humble 示例；使用 Jazzy 时将 humble 改为 jazzy。
source /opt/ros/humble/setup.bash
cd ~/lab3_ws

# 如果系统尚未初始化 rosdep，先执行 sudo rosdep init。
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

ros2 interface show chase_object_interfaces/msg/ObjectBearing
ros2 interface show chase_object_interfaces/msg/ObjectPolar
```

每个新终端都需要 source ROS 2 和本工作空间的 `install/setup.bash`。如果传感器在另一台计算机运行，还需要正确配置两端的 ROS 2 网络发现与 `ROS_DOMAIN_ID`。

## 3. 先核对传感器与坐标系

先按机器人已有方式启动底盘、相机、激光雷达以及底盘到雷达的 TF，再执行：

```bash
ros2 topic list -t
ros2 topic info /image_raw/compressed --verbose
ros2 topic echo /camera_info --once
ros2 topic echo /scan --once
ros2 topic info /cmd_vel --verbose
```

按实际话题名修改 `src/team_chase_object/config/lab3.yaml`，相机话题使用 `image_topic` 和 `camera_info_topic` 参数。`image_transport` 必须与消息类型一致；默认 compressed 对应 `CompressedImage`，如使用普通 `Image`，同时改用 raw。用上述输出中的 `header.frame_id` 确认相机与雷达坐标系名称。

TF 必须真实连接三个坐标系：激光扫描的 frame、相机 optical frame、机器人 `base_link`（或配置的底盘 frame）。相机 optical frame 的轴为右、下、前；常见底盘 frame 的轴为前、左、上。相机与雷达之间还存在安装位置与角度差，必须使用标定或机器人 URDF 中的真实外参。错误地使用单位变换会导致物体关联和转向错误。

启动文件根据实验机器人的安装位置提供 `base_scan` 到 `camera` 的静态 TF：平移为 `(0.07, 0.0, 0.05)` 米，四元数 `(x, y, z, w)` 为 `(-0.5, 0.5, -0.5, 0.5)`。这表示镜头位于雷达扫描原点前方 7 cm、上方 5 cm，并假设相机水平朝前、图像正立且未镜像。安装位置、朝向或坐标系名称改变时，需要修改 `launch/chase_object.launch.py` 中的 `camera_static_tf` 节点。

可用以下命令核对连通性，将占位符替换为实际 frame 名。使用启动文件自带的相机 TF 发布器时，在第 4 节启动后检查相机的 TF 连接：

```bash
ros2 run tf2_ros tf2_echo base_link ACTUAL_LASER_FRAME
ros2 run tf2_ros tf2_echo ACTUAL_CAMERA_OPTICAL_FRAME ACTUAL_LASER_FRAME
```

相机最好发布经过标定的 `CameraInfo`。没有可用内参时，代码可使用配置中的水平视场角近似计算方向；请填入实际镜头的水平视场角，并检查目标在画面左、中、右三个位置时的方向是否合理。目标需要覆盖激光雷达的水平扫描高度，否则相机看到了目标也无法获得它的真实距离。使用颜色明显、尺寸足够、表面能被激光可靠测到的目标，并避开同色背景。

测距节点通过 TF 将激光点投影到相机 optical frame，筛选视觉目标中心区域内的候选点，再从距离连续的点组中选择近处目标。默认要求至少两个有效激光点；目标过窄、过远或不在扫描平面内时可能无法通过筛选。输出距离和方向都位于配置的底盘坐标系，默认是 `base_link`。

关联只使用图像的水平方位，依赖目标确实与扫描平面相交；如果同一方向存在更近的遮挡物，系统可能把遮挡物当作目标。请使用能被雷达稳定测到的较大不透明目标，演示区域避免这类遮挡。默认距离指从底盘坐标原点到目标可见表面的平面距离，不是到目标几何中心或机器人保险杠的距离。

## 4. 启动与观察

在完成话题、底盘到雷达的 TF、相机安装参数和底盘接口核对后启动：

```bash
source /opt/ros/humble/setup.bash
source ~/lab3_ws/install/setup.bash
ros2 launch team_chase_object chase_object.launch.py
```

启动文件会让相机 TF 发布器与三个实验节点一起持续运行。使用前先停止之前手动运行的 `static_transform_publisher`。如果机器人 URDF 或其他发布器已经提供相机 TF，可关闭自带的发布器：

```bash
ros2 launch team_chase_object chase_object.launch.py publish_camera_tf:=false
```

使用自定义参数文件：

```bash
ros2 launch team_chase_object chase_object.launch.py \
  params_file:="$HOME/lab3_ws/src/team_chase_object/config/lab3.yaml"
```

仿真中仅当系统发布 `/clock` 时设置 `use_sim_time:=true`。实机保留默认 `false`。

在另一个终端观察完整链路：

```bash
ros2 topic echo /object/bearing
ros2 topic echo /object/polar
ros2 topic echo /cmd_vel
```

也可分别启动节点定位问题：

```bash
ros2 run team_chase_object detect_object --ros-args \
  --params-file "$HOME/lab3_ws/src/team_chase_object/config/lab3.yaml"
ros2 run team_chase_object get_object_range --ros-args \
  --params-file "$HOME/lab3_ws/src/team_chase_object/config/lab3.yaml"
ros2 run team_chase_object chase_object --ros-args \
  --params-file "$HOME/lab3_ws/src/team_chase_object/config/lab3.yaml"
```

上述三个命令各占用一个终端；以这种方式分别启动时，需要单独提供相机 TF。不要与完整 launch 同时运行，以免出现多个速度发布者。

## 5. 颜色、距离与控制调试

1. **颜色分割**：在实验实际光照下调整 `hsv_lower`、`hsv_upper`、`min_area` 与 `morph_kernel`。OpenCV 的 H 范围为 0–179，S、V 为 0–255。默认蓝色范围 `[100, 70, 60]` 到 `[130, 255, 255]` 只是起点。优先使用在这台相机、现场光照下调好的阈值；包含目标和背景的相机原始画面有助于细调范围。颜色判断必须用实时画面验证。同色物体过多时，应先整理实验环境。
2. **方向与测距**：先观察检测、测距输出。把目标放在正前方和左右两侧，确认方位、距离与实物一致；再缓慢移动目标，检查激光关联是否稳定。如果目标丢失或没有匹配激光点，先检查目标高度、TF、相机内参及时间同步。
3. **控制参数**：初始使用两个比例控制回路，分别控制线速度与角速度。配置中保留积分和微分项，默认从 `I=0、D=0` 开始调试。先以较低的速度上限调好转向，再调整距离回路；振荡时降低对应增益，必要时再谨慎增加微分项。积分项用于持续稳态误差时，需同时检查限幅与积分饱和保护。
4. **目标距离与停止误差**：按实验要求设置期望跟随距离及允许误差。确认目标过近时的行为与实验空间相适应，再逐步提高速度；不要直接把默认数值视为已适配实机。

主要控制参数如下，距离单位为米，角度单位为弧度：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `target_distance` | 0.60 | 期望目标距离 |
| `distance_deadband` / `bearing_deadband` | 0.05 / 0.04 | 距离、角度控制死区 |
| `linear_kp` / `linear_ki` / `linear_kd` | 0.5 / 0 / 0 | 距离 PID 参数 |
| `angular_kp` / `angular_ki` / `angular_kd` | 1.5 / 0 / 0 | 转向 PID 参数 |
| `max_forward_speed` / `max_reverse_speed` | 0.15 / 0.10 | 前进、后退速度上限，m/s |
| `max_angular_speed` | 0.8 | 角速度上限，rad/s |
| `heading_gate` | 0.6 | 方位偏差较大时先转向 |
| `control_rate_hz` | 20 | 控制发布频率 |
| `observation_timeout` | 0.5 | 目标数据失效超时，秒 |
| `min_confidence` | 0.2 | 最低目标置信度 |

这里的 `confidence` 是颜色轮廓面积占其包围框面积的比例，是检测质量指标，并不是经过统计标定的概率。可使用 `rqt_image_view` 查看 `/object/debug_image`（如未安装，另行安装对应 ROS 发行版的 `rqt_image_view`）；仅有订阅者时才生成调试图。

控制器对目标无效、输入超时或非有限值执行停车，速度也受配置上限约束。正常关闭节点时会尽力发送零速度，但强制终止进程或网络中断可能使停车消息无法到达底盘，实机必须配置底盘命令超时停车功能。本项目不实现障碍物避让。第一次测试请在空旷场地、有人可立即停车的条件下低速运行。

## 6. 验证与实验记录

在 Ubuntu 上构建后运行：

```bash
cd ~/lab3_ws
colcon test --packages-select team_chase_object
colcon test-result --verbose
```

### Ubuntu 上的 ROS 2 集成测试（无需机器人）

构建并加载工作空间后，运行合成数据端到端测试：

```bash
source /opt/ros/humble/setup.bash  # 使用 ROS 2 Jazzy 时改为 jazzy。
cd ~/lab3_ws
colcon build --symlink-install
source install/setup.bash
python3 src/team_chase_object/test/ros_smoke.py
```

脚本在同一进程启动三个节点，发布合成蓝色图像、相机内参、LiDAR 扫描和 TF，检查方位、距离、前进速度、传感器中断停车、目标重新出现后恢复，以及目标丢失停车。它把传感器、TF 和速度话题重映射到独立命名空间，不向机器人常规的 `/cmd_vel` 发布消息。此脚本需单独运行，不属于 `colcon test`。它不能验证硬件标定、真实传感器或手册要求的 5 秒实机表现。

只验证不依赖 ROS 的算法时，可在安装了 NumPy、OpenCV、pytest 的 Python 环境中运行：

```bash
cd ~/lab3_ws
PYTHONPATH=src/team_chase_object python3 -m pytest -q src/team_chase_object/test
```

本次 macOS 验证使用 Python 3.12、OpenCV 5.0.0、NumPy 2.5.3：**46 项测试通过**。测试包括图像压缩/分割、畸变与角度换算、带平移的坐标变换、雷达无效值/背景/扫描边界、两个 PID 控制回路、超时/重放保护，以及合成图像到理想机器人运动的整条算法链路。该结果不代表已经运行 ROS 节点或通过实机演示。

完成实机验证后再填写课程报告。建议记录以下场景的实际测量值：

| 场景 | 需要核对与记录的内容 |
|---|---|
| 目标在画面左、中、右 | 目标检测稳定，方向符号正确，雷达测距对应同一目标 |
| 目标静止于不同位置 | 收敛时间、最终距离误差、角度误差、是否振荡 |
| 目标缓慢移动 | 跟随是否稳定，速度是否在限制内 |
| 目标离开画面或被遮挡 | 速度及时归零，恢复检测后行为正常 |
| 相机、激光或目标消息中断 | 超时后停车，无旧目标继续驱动现象 |
| 光照或背景变化 | 误检、丢失和恢复情况 |

手册要求的 **5 秒内达到目标状态** 需要在真实机器人上测量，并根据实测结果调参。代码默认参数不能保证所有硬件、初始位置和场景均满足该指标。录像、截图、误差曲线和运行时间应来自实际实验，不应把示例或预期行为写成测得结果。

提交前核对：两位学生姓名、团队包名是否符合课程要求、全部源代码与消息定义、启动与参数文件，以及手册要求的报告回答和真实演示材料。

## 7. 实现公式与时间约定

对于无畸变针孔相机，检测中心像素为 `u`，水平焦距为 `fx`，主点为 `cx`：

```text
camera_bearing = atan2(cx - u, fx)          # 左侧为正，单位 rad
laser_angle_i = angle_min + i * angle_increment
p_laser = [r_i*cos(laser_angle_i), r_i*sin(laser_angle_i), 0]
p_camera = R_camera_laser * p_laser + t_camera_laser
point_bearing = atan2(-p_camera.x, p_camera.z)
```

有畸变参数时先对像素去畸变，再计算方位。只保留相机前方、位于目标水平角区间中央 `roi_fraction` 范围内的激光点；按相邻点间距分组，剔除过小点组，选择最近的连续点组。将组内点变换至底盘坐标系，分别对 x、y 取中位数得到目标位置：

```text
distance = hypot(x_base, y_base)
bearing = atan2(y_base, x_base)
e_distance = distance - target_distance
e_bearing = bearing
v = clip(PID_distance(e_distance), -max_reverse_speed, max_forward_speed)
w = clip(PID_bearing(e_bearing), -max_angular_speed, max_angular_speed)
```

进入对应死区时，该控制量准确置零并清空其积分/微分状态；方向误差达到 `heading_gate` 时先转向、暂停平移。PID 用控制定时器的实际稳态时钟间隔积分与微分，默认 I/D 为零。积分采用条件积分抗饱和，停止或丢失目标时重置。

摄像头和激光雷达根据源消息时间戳近似同步，默认最大差值 0.12 秒。输出时间戳保留两个输入中较早的时间，不能用处理完成时间掩盖旧数据。显式丢失目标后，队列里较旧的正检测不会重新启动机器人。控制端同时检查源时间戳和接收后经过的稳态时钟时间，拒绝重复/倒序时间戳；实际 ROS 时钟重置时才开始新的时间序列。

`20 Hz` 是控制指令发布频率，不等同于有效传感器更新率。报告中的采样时间应通过实际相机、`/scan` 和 `/object/polar` 更新间隔测量；可使用 `ros2 topic hz`，并记录抖动和端到端延迟。跨电脑运行时，系统时钟需要同步，否则时间戳检查会触发停车。

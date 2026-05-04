# RobotSim: Alicia-D 真实物理抓取仿真

RobotSim 是一个基于 ROS 2 Humble、Gazebo Fortress 和 MoveIt2 的 Alicia-D 六自由度机械臂抓取仿真项目。当前主线目标是验证三物体顺序抓取闭环：

```text
red_cylinder -> yellow_box -> brown_cube
```

系统使用顶视 RGB-D 相机做颜色感知，用 MoveIt2 规划 Alicia-D 机械臂运动，用 Gazebo 的 collision、contact 和 friction 模拟真实物理夹取。旧的虚拟附着模式仍保留，可作为调试 fallback。

![RobotSim 仿真运行截图](media/simulation.png)

## 当前场景

取料区已经移除 Bin A，物体直接放在地面上，以降低下爪碰撞难度。放置区 Bin B 保留，但墙高降为原先一半。

三个物体的 SDF 模型和默认初始位姿如下：

| Target | Gazebo model | Geometry | Pose |
| --- | --- | --- | --- |
| `red_cylinder` | `test_can` | radius `0.035`, height `0.11` | `(-0.30, 0.255, 0.055)` |
| `yellow_box` | `test_box` | box `0.06 x 0.10 x 0.07` | `(-0.36, 0.10, 0.035)`, yaw `1.5708` |
| `brown_cube` | `test_brown_cube` | box `0.065 x 0.065 x 0.065` | `(-0.24, 0.10, 0.0325)` |

三件物体两两边到边间距 ≥3.7cm，避开了下爪时夹爪 pad 与邻居的横向碰撞。所有物体的中心距离基座原点 ≤0.40m，落在机械臂可达半径内。

Bin B 的默认多 slot 放置点：

```text
red_cylinder: (-0.36, -0.145, 0.28)
yellow_box:   (-0.31, -0.145, 0.28)
brown_cube:   (-0.26, -0.145, 0.28)
```

## 环境

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress / Ignition
- MoveIt2
- Python 3.10

常用依赖：

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  ros-humble-moveit \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-controller-manager \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-tf2-ros

pip3 install pymoveit2
```

## 编译

```bash
cd ~/RobotSim
source setup_env.sh
colcon build --symlink-install
source install/setup.bash
```

静态检查：

```bash
python3 -m py_compile src/perception_mvp/perception_mvp/*.py src/perception_mvp/launch/task_demo.launch.py
python3 -c "import xml.etree.ElementTree as ET; ET.parse('alicia_gz_sim/worlds/bin_scene.sdf')"
ign sdf -k alicia_gz_sim/worlds/bin_scene.sdf
xacro src/alicia_moveit_config/config/Alicia_D_v5_6_gripper_100mm.urdf.xacro
```

## 启动仿真

建议每次启动前清理残留进程：

```bash
cd ~/RobotSim
source setup_env.sh
source install/setup.bash
bash cleanup.sh
```

Headless Gazebo + MoveIt2：

```bash
ros2 launch alicia_moveit_config sim_demo.launch.py \
  use_rviz:=false \
  headless:=true \
  render_engine:=auto
```

如果 Nano 已经接显示器，需要本地看 RViz：

```bash
ros2 launch alicia_moveit_config sim_demo.launch.py \
  use_rviz:=true \
  headless:=false \
  render_engine:=auto
```

等待 `move_group` 日志出现 `You can start planning now!` 后，再启动任务。

## 真实物理抓取

多物体闭环：

```bash
ros2 launch perception_mvp task_demo.launch.py \
  target_sequence:="['red_cylinder','yellow_box','brown_cube']" \
  grasp_mode:=physical \
  grasp_style:=auto
```

单物体调试：

```bash
ros2 launch perception_mvp task_demo.launch.py \
  target_sequence:="['red_cylinder']" \
  grasp_mode:=physical \
  grasp_style:=top_down
```

`grasp_mode:=physical` 不会启动 `virtual_grasp_node`，日志中也不应出现 `Attached target=...`。物体移动应来自夹爪 collision、Gazebo contact/friction 和夹爪 effort 控制。

当前 `physical + auto` 会解析为 `top_down`，但这里的 top_down 不是严格竖直 wrist 姿态，而是 Alicia-D 当前 IK 更容易到达的上方夹取姿态：

```text
grasp_rpy_deg = [180.0, 90.0, 0.0]
grasp_reference_offset_xyz = [0.0, 0.0, 0.0]
gripper_close_position = 0.040
goal_pose_joint_fallback_min_z = -1.0
```

默认按目标的下探深度：

```text
red_cylinder: -0.075
yellow_box:   -0.040
brown_cube:   -0.040
```

这些值可以在 launch 时覆盖：

```bash
ros2 launch perception_mvp task_demo.launch.py \
  target_sequence:="['red_cylinder','yellow_box','brown_cube']" \
  grasp_mode:=physical \
  grasp_height_offsets:="[-0.075,-0.040,-0.040]" \
  gripper_close_position:=0.040 \
  place_slots_xyz:="[-0.36,-0.145,0.28,-0.31,-0.145,0.28,-0.26,-0.145,0.28]"
```

## 严格竖直诊断姿态

严格竖直姿态保留为诊断模式：

```bash
ros2 launch perception_mvp task_demo.launch.py \
  target_sequence:="['red_cylinder']" \
  grasp_mode:=physical \
  grasp_style:=strict_top_down
```

在当前 Alicia-D IK 和默认场景下，`strict_top_down` 使用 `grasp_rpy_deg=[180.0, 0.0, 90.0]`，验证时 pregrasp 规划会 `STATUS_ABORTED`，因此默认仍使用可达的 `top_down=[180.0,90.0,0.0]`。

## 虚拟附着 fallback

虚拟模式用于对比状态机和感知流程，不代表真实物理抓取：

```bash
ros2 launch perception_mvp task_demo.launch.py \
  target_sequence:="['red_cylinder','yellow_box','brown_cube']" \
  grasp_mode:=virtual
```

此模式会启动 `virtual_grasp_node`，在夹爪闭合后用 Gazebo `set_pose` 让物体跟随夹爪。

## 主要节点

```text
color_perception_node
  订阅 RGB-D 图像，按当前目标颜色检测 red_cylinder / yellow_box / brown_cube。

move_to_pregrasp_node
  接收状态机触发，执行 MoveIt2 pregrasp、grasp、lift、place 和 gripper action。

task_state_machine_node
  按 target_sequence 顺序执行 detect -> pregrasp -> grasp_approach (gripper 部分收窄)
  -> descend -> close -> lift -> place -> open。

virtual_grasp_node
  仅 virtual 模式启动，用 set_pose 模拟附着。
```

## 抓取下爪前的夹爪收窄

为了减少下爪过程中夹爪手指 pad 与邻居物体的横向碰撞，状态机在 `VERIFY_PREGRASP` 之后插入了一个
`GRIPPER_APPROACH` 阶段，把夹爪从全开 (`Gripper=0.0`，pad 横向跨度 ≈ 15.5cm) 收到 `gripper_approach_position`
(默认 `0.010`，pad 跨度 ≈ 13.5cm；指间间隙仍有 ≈ 8.3cm，足以包住直径 7cm 的圆柱)。

可在 launch 时覆盖：

```bash
ros2 launch perception_mvp task_demo.launch.py \
  target_sequence:="['red_cylinder','yellow_box','brown_cube']" \
  grasp_mode:=physical \
  gripper_approach_position:=0.012
```

## 在 Jetson Nano 上看 RViz

Nano 接显示器后, 把 `use_rviz:=true` 让 RViz 直接在本地启动。Gazebo GUI 在 Nano 上比较吃 GPU,
仍然推荐 `headless:=true` 让 Gazebo 跑 server-only, 由 RViz 负责可视化。

```bash
cd ~/RobotSim
source setup_env.sh
source install/setup.bash
bash cleanup.sh

ros2 launch alicia_moveit_config sim_demo.launch.py \
  use_rviz:=true \
  headless:=true \
  render_engine:=auto
```

看到 `You can start planning now!` 后, 在另一个终端起任务:

```bash
cd ~/RobotSim
source setup_env.sh
source install/setup.bash

ros2 launch perception_mvp task_demo.launch.py \
  target_sequence:="['red_cylinder','yellow_box','brown_cube']" \
  grasp_mode:=physical \
  grasp_style:=top_down
```

RViz 中默认就有 `RobotModel`、`PlanningScene`、`TF` 显示项。手动加这几个看抓取闭环很直观:

- `Marker` → topic `/perception/target_marker`, 每个目标会出现一个彩色小球, 颜色对应 red/yellow/brown
- `MotionPlanning` 面板 → Planning Group 选 `Alicia` 可以看 MoveIt 规划轨迹
- 跟进状态机: 终端里 `ros2 topic echo /task/state` 和 `ros2 topic echo /task/status_text`

如果 Nano 没接显示器, 远端 SSH 看 RViz 推荐用 VNC 而不是 X-forwarding —— RViz 的 OpenGL 上下文在
X-forwarding 下经常崩。VNC 服务跑在 Nano 上, 用客户端连进去看本机渲染的 RViz 即可。

## 常用观察命令

```bash
ros2 topic echo /task/state
ros2 topic echo /task/status_text
ros2 topic echo /perception/current_target_point_world
ros2 topic echo /joint_states --once
ros2 control list_controllers
```

检查 Gazebo 里有哪些 topic：

```bash
ign topic -l
```

## 调参顺序

如果 pregrasp 失败，先把物体向工作区中心移动，优先保持 `x` 在 `-0.38` 到 `-0.25`、`y` 在 `0.10` 到 `0.21`。

如果下探失败，先允许或保留 joint fallback：

```bash
goal_pose_joint_fallback_min_z:=-1.0
```

如果夹到但滑落，按顺序试：

```text
gripper_close_position: 0.040 -> 0.045
grasp_height_offsets: 单个目标再下探 0.005m
finger/object friction: 最后再继续提高
```

如果物体被一侧手指推走，优先调：

```bash
grasp_reference_offset_xyz:="[0.0,0.0,0.0]"
grasp_position_offset_xyz:="[0.0,0.0,0.0]"
```

最后更新：2026-05-03

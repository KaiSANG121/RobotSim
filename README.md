# Alicia 机器人智能抓取仿真系统 v2.1

> **环境**: WSL2 Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress (Ignition)

纯 Alicia-D 6-DOF 机械臂 + 原始 100mm 平行夹爪的 PDE 闭环仿真系统。

---

## 系统架构

```
Gazebo Sim
  └── Alicia-D 6-DOF + 夹爪 (gz_ros2_control)
  └── 俯视 RGB-D 相机 (静态)
  └── 3 个测试物体 (动态)
        │
        ├── ROS 2 节点
        │     ├── color_perception_node   (感知层)
        │     ├── task_state_machine_node (决策层)
        │     └── move_to_pregrasp_node   (执行层)
        │
        └── MoveIt2 move_group
```

---

## 目录结构

```
~/RobotSim/
├── setup_env.sh                      # 环境配置脚本
├── README.md                         # 本文件
├── src/
│   ├── alicia_d_descriptions/        # 机器人 URDF + Mesh 描述包
│   ├── alicia_moveit_config/         # MoveIt2 配置包（含 sim_demo.launch.py）
│   └── perception_mvp/               # 感知+决策+执行 Python 包
└── alicia_gz_sim/                    # Gazebo 仿真资产（非 ROS 包）
    ├── config/
    ├── worlds/
    └── models/
        └── alicia_d_fixed/           # 纯 Alicia-D Gazebo 模型
```

---

## 快速开始

### 1. 环境准备

```bash
# 安装依赖
sudo apt-get update && sudo apt-get install -y \
  ros-humble-moveit \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-controller-manager \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  python3-colcon-common-extensions

# WSL2 集显软件渲染必需 (默认只装了 ogre2，软件渲染会崩)
sudo apt-get install -y libignition-rendering6-ogre

pip3 install pymoveit2
```

> **WSL2 + 集显说明**: `bin_scene.sdf` 默认配置为 ogre1 渲染引擎，`setup_env.sh` 会自动设置软件渲染环境变量。RGBD 相机已拆为独立 RGB + Depth 两个 sensor（topic 命名向后兼容）。如果你的环境有可用 GPU 直通（`ls -la /dev/dri/renderD128` 当前用户可读），可以把 sensors plugin 改回 `ogre2` 并取消 `setup_env.sh` 中的软件渲染段。

### 2. 复制 Mesh 文件（⚠️ 重要）

STL mesh 文件**不包含**在本仓库中，需从旧项目手动复制：

```bash
# 从旧 alicia_ws 复制 mesh
mkdir -p ~/RobotSim/src/alicia_d_descriptions/meshes/Alicia_D_v5_6/gripper_100mm/

# 假设旧项目在 ~/alicia_ws
cp ~/alicia_ws/src/alicia_d_descriptions/meshes/Alicia_D_v5_6/gripper_100mm/*.STL \
   ~/RobotSim/src/alicia_d_descriptions/meshes/Alicia_D_v5_6/gripper_100mm/
```

需要的 STL 文件：
- `base_link.STL`
- `Link1.STL` ~ `Link6.STL`
- `left_gripper.STL`
- `right_gripper.STL`

### 3. 构建工作空间

```bash
cd ~/RobotSim
source setup_env.sh
colcon build --symlink-install
source install/setup.bash
```

### 4. 启动仿真

> ⚠️ **每次启动前必须先 cleanup**。launch 在 Ctrl-C 关闭时偶发 `move_group` 析构 segfault，会留下残留 `controller_manager` / `move_group` 进程；下次启动时新旧 controller_manager 同时存在，导致 spawner 全部失败、控制器无法 active。

```bash
cd ~/RobotSim
source setup_env.sh

# 启动前清理上次残留进程 (必做)
bash ~/RobotSim/cleanup.sh

# 默认 headless 模式: Gazebo server-only, 不开 GUI 窗口 (WSL2 集显友好)
# 可视化通过 RViz2 完成
ros2 launch alicia_moveit_config sim_demo.launch.py

# 如果有 GPU 直通可用，可以打开 Gazebo 主窗口
ros2 launch alicia_moveit_config sim_demo.launch.py headless:=false
```

> **关于 headless 模式**: WSL2 集显走 Mesa llvmpipe 软件渲染时，Gazebo 主窗口的 Qt5 + OGRE 上下文会在 `QOpenGLContext::doneCurrent()` 处 segfault。`headless:=true`（默认）跳过 Gazebo GUI，物理仿真和 RGBD 传感器照常工作，**RViz2 来承担所有可视化**（机器人模型、相机图像、MoveIt 规划、TF 树都能在 RViz 看）。

### 5. 启动感知与状态机（新终端）

> ⚠️ **必须传 `use_sim_time:=true` 参数**：所有节点都要订阅 Gazebo 仿真时间。否则 MoveIt 收到的 `/joint_states` 时间戳是 sim_time（从 0 开始），节点本身用 wall clock，二者相差 56 年，MoveIt 会拒绝接受当前关节状态并 abort 所有轨迹执行。

```bash
source ~/RobotSim/setup_env.sh

# 感知节点
ros2 run perception_mvp color_perception_node --ros-args -p use_sim_time:=true

# 状态机（新终端）
ros2 run perception_mvp task_state_machine_node --ros-args -p use_sim_time:=true

# 执行节点（新终端）
ros2 run perception_mvp move_to_pregrasp_node --ros-args -p use_sim_time:=true
```

---

## 验收检查

```bash
# 检查关节状态
ros2 topic echo /joint_states

# 检查控制器状态
ros2 control list_controllers

# 检查 TF 链
ros2 run tf2_ros tf2_echo world base_link

# 检查相机 TF
ros2 run tf2_ros tf2_echo world rgbd_camera_frame

# 检查相机图像频率
ros2 topic hz /rgbd_camera/image
```

---

## Topic 接口

### 感知层输出
| Topic | 类型 | 描述 |
|-------|------|------|
| `/perception/current_target_visible` | Bool | 目标是否可见 |
| `/perception/current_target_point_world` | PointStamped | 目标世界坐标 |
| `/perception/current_target_pregrasp_world` | PointStamped | 预抓取位置（上方 0.10m）|
| `/perception/current_target_name_echo` | String | 当前跟踪目标名 |

### 决策层输出
| Topic | 类型 | 描述 |
|-------|------|------|
| `/task/current_target_name` | String | 指示感知关注目标 |
| `/task/enable_pregrasp` | Bool | 触发 pregrasp 运动 |
| `/task/goal_pose_world` | PoseStamped | 放置目标位姿 |
| `/task/execute_goal_pose` | Bool | 触发放置运动 |
| `/task/gripper_command` | Float64 | 夹爪开合（0.0=开, 0.05=闭）|

---

*版本: v2.1 | 最后更新: 2026-04-26*

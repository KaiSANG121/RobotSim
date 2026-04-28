# Codex 工作总结

日期：2026-04-28

本文档总结 Claude 之后，Codex 在 `RobotSim` 项目中继续完成的排查、代码修改、验证结果，以及后续仍需要修复或确认的问题。

## 1. 接手时的项目状态

Claude 之前已经把 PDE 闭环中的物理、控制、MoveIt 执行主链路打通到比较接近可跑的状态。根据当时的上下文：

- MoveIt/控制器链路本身可以执行，`failure.md` 中的手动 Plan + Execute 已经成功。
- 状态机中已经包含 `/task/gripper_command` 发布逻辑。
- `model.sdf` 中重复控制器插件冲突的问题已经被处理。
- 当时的主要问题不是 MoveIt 完全不可用，而是完整闭环里状态机没有稳定进入或正确完成预抓取执行。

Codex 接手后遇到两个连续问题：

1. 感知闭环不触发预抓取：`/rgbd_camera/camera_info` 或 `/rgbd_camera/depth_image` 没有进入感知节点，导致感知节点不发布 `/perception/current_target_visible` 和 `/perception/current_target_pregrasp_world`。
2. 感知触发后，MoveIt 预抓取 abort：状态机已经进入 `PLAN_PREGRASP`，但执行节点请求的目标点明显不合理，例如 `z≈0.974`，MoveIt 报 `Unable to sample any valid states for goal tree`。

## 2. 已完成的主要工作

### 2.1 修复相机桥接配置

新增文件：

- `src/alicia_moveit_config/config/ros_gz_bridge.yaml`

修改文件：

- `src/alicia_moveit_config/launch/sim_demo.launch.py`

原来的 launch 中依赖 `parameter_bridge` 的命令行 topic 参数和 node-level remap，把 Gazebo 侧的 camera info 试图重映射成 ROS 侧 `/rgbd_camera/camera_info`。这个方式对 `ros_gz_bridge` 内部 advertise 的 topic 不够可靠，而且一开始怀疑的 Gazebo topic 名 `/rgbd_camera/image/camera_info` 也不是实际发布的 topic。

Codex 做了两步修复：

1. 改用 YAML `config_file` 显式配置 `ros_gz_bridge`，避免继续依赖 node-level remap。
2. 通过 `ign topic` 检查 Gazebo 实际发布的 topic，确认相机相关 GZ topics 是：
   - `/rgbd_camera/image`
   - `/rgbd_camera/camera_info`
   - `/rgbd_camera/depth_image`

最终 `ros_gz_bridge.yaml` 中显式桥接：

- ROS `/rgbd_camera/image` <- GZ `/rgbd_camera/image`
- ROS `/rgbd_camera/camera_info` <- GZ `/rgbd_camera/camera_info`
- ROS `/rgbd_camera/depth_image` <- GZ `/rgbd_camera/depth_image`
- ROS `/world/default/model/alicia_d_fixed/joint_state` <- GZ 同名 topic
- ROS `/clock` <- GZ `/clock`

对应地，`sim_demo.launch.py` 里的 `ros_gz_bridge` 节点改为：

- 使用 `parameters=[{'config_file': bridge_config_file}]`
- 移除 `/rgbd_camera/image/camera_info -> /rgbd_camera/camera_info` remapping

这个修改的目标是让感知节点继续使用稳定的 canonical ROS topic：`/rgbd_camera/camera_info`，而不是让感知节点直接依赖 Gazebo/raw topic 名。

### 2.2 给感知节点增加诊断与 QoS 兼容

修改文件：

- `src/perception_mvp/perception_mvp/color_perception_node.py`

已完成改动：

- 引入 `time`，用于节流诊断日志。
- 引入 `qos_profile_sensor_data`，RGB、depth、camera_info 三个订阅统一使用 sensor-data QoS，更兼容仿真传感器常见的 BEST_EFFORT/RELIABLE 发布端。
- 在 `process()` 开头增加 5 秒节流 warn。当 RGB、depth、camera_info 三个 cache 任意一个为空时，会明确打印缺失的 topic，例如：

```text
process() idling, missing topic(s): /rgbd_camera/camera_info (throttled to 5s; check `ros2 topic hz` on these)
```

这解决了之前“感知节点静默 idle，但不知道缺哪一路数据”的问题。

### 2.3 修正感知使用的相机坐标系

修改文件：

- `src/perception_mvp/perception_mvp/color_perception_node.py`
- `src/alicia_moveit_config/launch/sim_demo.launch.py`

原问题：

- Gazebo 的 `CameraInfo.header.frame_id` 类似 `overhead_rgbd_camera::camera_link::depth_camera`。
- 感知节点最初如果直接使用这个 frame id，既不一定和 TF 树中的 frame 对齐，也容易因为 Gazebo scoped name 和 ROS TF frame 命名不一致导致 transform 问题。
- 后续虽然打通了感知，但静态 TF 的方向不对，把相机光学深度轴映射到了 world 的错误方向，导致目标点被变换到类似 `(0.370, 0.159, 0.974)` 这种明显不在 Bin A 附近的位置。

Codex 做了以下修复：

- 在感知节点中新增参数 `camera_frame`，默认值为 `rgbd_camera_frame`。
- 感知节点构造 `PointStamped` 时使用 `camera_frame`，不再直接依赖 Gazebo message header 中的 scoped frame id。
- 如果用户显式把 `camera_frame` 设置为空字符串，才回退使用消息 header，并把 `::` 替换成 `/`。
- 在 `sim_demo.launch.py` 中把 `world -> rgbd_camera_frame` 静态 TF 改为 ROS 新式参数格式，并设置为标准光学坐标含义：
  - translation: `x=-0.36, y=0.18, z=0.85`
  - rotation: `roll=pi, pitch=0, yaw=-pi/2`
  - 语义：相机 `+Z` 是深度方向，在 world 中朝下；`+X` 是图像右侧；`+Y` 是图像下方。

这项修复的预期结果是：

- 红色圆柱的世界坐标应回到 Bin A 附近，例如 `x≈-0.35, y≈0.20, z≈0.05~0.13`。
- 预抓取点应在物体上方约 `0.10m`，而不是出现在 `z≈0.97` 的相机附近高处。

### 2.4 给感知目标点增加 sanity check

修改文件：

- `src/perception_mvp/perception_mvp/color_perception_node.py`

为了避免错误坐标再次直接进入 MoveIt，Codex 给感知节点增加了 Bin A 工作区保护：

- 新增参数：
  - `bin_a_workspace_min = [-0.55, 0.05, -0.05]`
  - `bin_a_workspace_max = [-0.15, 0.34, 0.35]`
- 新增 `point_in_bin_a_workspace()` 检查。
- 当 transform 后的世界坐标明显不在 Bin A 工作区时：
  - 打印 warn，包含 `camera_xyz`、`world_xyz` 和工作区上下界。
  - 发布 `visible=False`。
  - 不发布错误 pregrasp 给执行节点。

这项改动的作用是把错误尽量停在感知层，而不是把明显不合理的目标 pose 继续送进 MoveIt。

### 2.5 修复执行节点“Abort 仍反馈成功”的问题

修改文件：

- `src/perception_mvp/perception_mvp/move_to_pregrasp_node.py`

原问题：

- `pymoveit2.wait_until_executed()` 已经可以返回执行是否成功。
- 旧代码调用了 `wait_until_executed()`，但没有检查返回值。
- 所以即使 MoveIt action 返回 `STATUS_ABORTED`，执行节点仍可能返回成功，导致状态机误以为 `Pregrasp success`，继续进入 `VERIFY_PREGRASP`、`VIRTUAL_GRASP` 和 place 阶段。

Codex 修复了两处执行路径：

- `_execute_pregrasp()`
- `_execute_goal_pose()`

现在逻辑为：

- `ok = self.moveit2.wait_until_executed()`
- 如果 `ok == False`，日志明确打印 motion aborted/failed，并返回 `False`。
- 只有 `ok == True` 时，才打印 completed successfully，并向状态机反馈成功。

这个修改解决的是状态机反馈语义错误。也就是说，MoveIt 如果仍然因为 IK 或规划失败 abort，状态机不会再误判为成功。

### 2.6 同步 Gazebo world 注释

修改文件：

- `alicia_gz_sim/worlds/bin_scene.sdf`

Codex 根据实际 `ign topic` 检查结果，把相机 topic 相关注释同步为真实 topic，避免后续再被 `/rgbd_camera/image/camera_info` 这个错误猜测误导。

## 3. 已验证的内容

Codex 已执行过以下验证：

```bash
colcon build --symlink-install --packages-select alicia_moveit_config perception_mvp
```

构建通过。

```bash
ros2 launch alicia_moveit_config sim_demo.launch.py --show-args
```

launch 参数解析通过。由于沙箱环境下默认 `~/.ros/log` 可能不可写，验证时使用过临时 ROS log 目录。

```bash
python3 -m py_compile \
  src/perception_mvp/perception_mvp/color_perception_node.py \
  src/perception_mvp/perception_mvp/move_to_pregrasp_node.py
```

两个 Python 节点语法检查通过。

通过 Gazebo topic 检查确认：

- `/rgbd_camera/camera_info` 存在，并且能 echo 到 CameraInfo。
- `/rgbd_camera/depth_image` 存在，并且能 echo 到 depth Image。
- `/rgbd_camera/image` 存在。

用户侧也已经验证过，在桥接问题修复后：

- 感知节点能够起来。
- 执行节点 ready。
- 状态机能够进入 `PLAN_PREGRASP`。
- 状态机能触发执行节点 `enable_pregrasp received`。

这说明“状态机完全不进入预抓取”的第一阶段问题已经被打通。

## 4. 当前已经解决的问题

已解决或已完成代码修复的问题：

1. 感知节点缺少 cache 时没有足够诊断信息。
   - 现在会明确提示缺 RGB、depth、camera_info 中的哪一路。

2. `camera_info` topic 桥接方式不稳，且曾使用错误 GZ topic 名。
   - 现在改为 YAML 显式桥接，最终 GZ topic 使用 `/rgbd_camera/camera_info`。

3. 感知节点直接依赖 Gazebo scoped frame id 的风险。
   - 现在默认使用 `rgbd_camera_frame`。

4. 相机静态 TF 方向错误导致目标点跑到错误世界坐标。
   - 现在 `world -> rgbd_camera_frame` 改为符合光学坐标语义的配置。

5. 错误目标点直接送入 MoveIt。
   - 现在加入 Bin A workspace sanity check，异常坐标会发布 `visible=False` 并 warn。

6. MoveIt abort 后执行节点仍反馈成功。
   - 现在 `_execute_pregrasp()` 和 `_execute_goal_pose()` 会检查 `wait_until_executed()` 返回值，失败时返回 `False`。

## 5. 目前仍需要继续确认或修复的问题

### 5.1 必须重启所有节点后验证最终 TF 修复

静态 TF 是在 launch 启动时发布的。修改 `sim_demo.launch.py` 后，需要完整 cleanup 并重启 launch，否则旧 TF 仍可能在当前运行图里生效。

下一步需要确认：

```bash
ros2 topic echo /perception/current_target_point_world --once
ros2 topic echo /perception/current_target_pregrasp_world --once
```

预期：

- target point 应接近 Bin A，例如红色圆柱大约 `x=-0.35, y=0.20, z=0.05~0.13`。
- pregrasp point 应在目标上方约 `0.10m`。
- 不应该再出现 `z≈0.97` 这种相机高度附近的目标点。

### 5.2 如果目标点正确但 MoveIt 仍 abort，需要做第二阶段 IK/姿态调试

如果修正 TF 后，目标点已经合理，但 MoveIt 仍报：

```text
Unable to sample any valid states for goal tree
```

那问题才进入第二阶段：夹爪目标姿态、orientation tolerance、pregrasp 高度或 IK 可达性。

可调方向包括：

- 放宽 `orientation_tolerance`，例如从 `0.05` 调到 `0.15`。
- 对 pregrasp yaw/roll 做候选姿态搜索。
- 调整 pregrasp z offset，避免目标太低或太贴近料仓边界。
- 用 RViz MotionPlanning 对同一目标 pose 做手动 IK/Plan 复核。

当前不建议优先改夹爪姿态，因为之前 abort 的直接原因是目标点本身明显错误。

### 5.3 如果目标仍不可见，需要进入 HSV/深度调试

如果三路相机 cache 都正常，但 `/perception/current_target_visible` 仍为 false，则需要进入颜色阈值和深度图第二阶段调试：

- 检查红色圆柱在 RGB 图像中的 HSV 是否落在阈值内。
- 检查 depth ROI 是否有效，不是 NaN/Inf/0。
- 观察调试图或临时打印 mask 面积、中心像素、depth median。

这不是桥接/TF 修复的前置问题，而是感知识别策略问题。

### 5.4 Place 阶段仍需要完整闭环验证

用户之前的日志中，状态机在错误 pregrasp success 后进入了：

- `VIRTUAL_GRASP`
- `MOVE_TO_PLACE_HOVER`
- `WAIT_PLACE_RESULT`

但那是在执行节点误报成功的情况下发生的。修复后需要重新验证：

- pregrasp 真实成功后才会进入 place。
- place hover pose 是否可达。
- `/task/place_running` 和 `/task/place_success` 反馈是否符合真实 MoveIt 结果。

### 5.5 当前抓取仍偏“虚拟闭环”，物理夹持/附着还需要后续增强

当前状态机有夹爪命令，能够发 `/task/gripper_command`，但完整物理抓取是否稳定还取决于：

- Gazebo 中夹爪与物体的接触参数。
- 是否需要 attach/link constraint 或其他虚拟抓取机制。
- 物体是否能跟随机械臂移动到放置区。

这部分不属于本次 camera_info、TF 和执行反馈修复的直接范围，但会影响最终“抓起并放下”的演示效果。

## 6. 建议的下一轮测试顺序

完整重启前先构建：

```bash
cd ~/RobotSim
source setup_env.sh
colcon build --symlink-install --packages-select alicia_moveit_config perception_mvp
source install/setup.bash
```

清理旧进程：

```bash
bash ~/RobotSim/cleanup.sh
```

终端 1：启动仿真、控制器、MoveIt、RViz、桥接和静态 TF：

```bash
cd ~/RobotSim
source setup_env.sh
source install/setup.bash
ros2 launch alicia_moveit_config sim_demo.launch.py
```

终端 2：检查相机桥接：

```bash
cd ~/RobotSim
source setup_env.sh
source install/setup.bash

ros2 topic hz /rgbd_camera/image
ros2 topic hz /rgbd_camera/depth_image
ros2 topic hz /rgbd_camera/camera_info
ros2 topic info /rgbd_camera/camera_info --verbose
```

终端 3：启动感知节点：

```bash
cd ~/RobotSim
source setup_env.sh
source install/setup.bash

ros2 run perception_mvp color_perception_node --ros-args -p use_sim_time:=true
```

然后检查目标点：

```bash
ros2 topic echo /perception/current_target_visible --once
ros2 topic echo /perception/current_target_point_world --once
ros2 topic echo /perception/current_target_pregrasp_world --once
```

终端 4：启动执行节点：

```bash
cd ~/RobotSim
source setup_env.sh
source install/setup.bash

ros2 run perception_mvp move_to_pregrasp_node --ros-args -p use_sim_time:=true
```

终端 5：启动状态机：

```bash
cd ~/RobotSim
source setup_env.sh
source install/setup.bash

ros2 run perception_mvp task_state_machine_node --ros-args -p use_sim_time:=true
```

验收重点：

- 感知节点不再持续打印 missing cache。
- `/perception/current_target_point_world` 位于 Bin A 工作区。
- `/perception/current_target_pregrasp_world` 不再接近相机高度。
- 执行节点如果 MoveIt abort，会明确发布失败，不会误报成功。
- MoveIt 不再因错误目标点报 `Unable to sample any valid states for goal tree`。
- 机械臂在 RViz/Gazebo 中实际移动到 pregrasp。

## 7. 修改文件清单

Codex 本轮涉及的文件：

- `src/alicia_moveit_config/config/ros_gz_bridge.yaml`
  - 新增桥接 YAML，显式配置相机、joint_state 和 clock 的 GZ_TO_ROS 桥接。

- `src/alicia_moveit_config/launch/sim_demo.launch.py`
  - `ros_gz_bridge` 改用 `config_file`。
  - 移除 camera_info node-level remap。
  - 修正 `world -> rgbd_camera_frame` 静态 TF。

- `src/perception_mvp/perception_mvp/color_perception_node.py`
  - 增加 sensor-data QoS。
  - 增加 cache 缺失节流日志。
  - 增加 `camera_frame` 参数。
  - 增加 Bin A workspace sanity check。
  - 增加异常坐标 warn，避免错误目标进入 MoveIt。

- `src/perception_mvp/perception_mvp/move_to_pregrasp_node.py`
  - 检查 `wait_until_executed()` 返回值。
  - 修复 MoveIt abort 后误报 success 的问题。

- `alicia_gz_sim/worlds/bin_scene.sdf`
  - 同步相机 topic 注释，记录真实 Gazebo topic 名。

## 8. 当前结论

目前第一阶段“感知闭环不触发预抓取”的问题已经从桥接、QoS、诊断和 topic 命名层面完成修复，并且状态机已经能进入 `PLAN_PREGRASP`。

第二阶段“预抓取 abort”的直接根因被定位为相机 TF 坐标方向错误导致目标点不合理，同时执行节点存在失败反馈 bug。Codex 已经修正静态 TF，并修复执行节点对 MoveIt abort 的返回值处理。

下一步最关键的是完整重启后验证最终坐标：

- 如果目标点回到 Bin A 且 MoveIt 成功，闭环即可继续向 place 和物理抓取验证推进。
- 如果目标点正确但 MoveIt 仍 abort，再进入夹爪姿态、IK、tolerance 和 pregrasp offset 的第二阶段调试。
- 如果目标不可见，则进入 HSV/depth 识别参数调试。

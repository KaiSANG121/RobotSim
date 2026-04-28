# Mesh 文件目录

本目录应包含以下 STL 文件（从 Alicia-D 原始项目复制）：

- `base_link.STL`
- `Link1.STL`
- `Link2.STL`
- `Link3.STL`
- `Link4.STL`
- `Link5.STL`
- `Link6.STL`
- `left_gripper.STL`
- `right_gripper.STL`

## 复制命令

```bash
# 从旧 alicia_ws 项目复制（请根据实际路径调整）
cp ~/alicia_ws/src/alicia_d_descriptions/meshes/Alicia_D_v5_6/gripper_100mm/*.STL \
   ~/RobotSim/src/alicia_d_descriptions/meshes/Alicia_D_v5_6/gripper_100mm/
```

这些 STL 文件是 Gazebo 可视化和碰撞检测所必需的。

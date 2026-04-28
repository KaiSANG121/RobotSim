#!/bin/bash
# 启动 sim_demo.launch.py 之前先跑一遍, 清理上次未干净退出的残留进程。
# 残留 controller_manager / move_group 会污染 ROS 域,
# 导致 spawner 看到 "Controller already loaded" 报错全部 die。
#
# 用法: bash ~/RobotSim/cleanup.sh

set +e  # 进程不存在时 pkill 返回非零, 不要让 set -e 中断脚本

echo "[cleanup] killing leftover sim processes..."

pkill -9 -f "ign gazebo"               2>/dev/null
pkill -9 -f "gz_sim"                   2>/dev/null
pkill -9 -f "gzserver"                 2>/dev/null
pkill -9 -f "ros_gz_sim"               2>/dev/null
pkill -9 -f "parameter_bridge"         2>/dev/null
pkill -9 -f "controller_manager"       2>/dev/null
pkill -9 -f "ros2_control_node"        2>/dev/null
pkill -9 -f "spawner"                  2>/dev/null
pkill -9 -f "move_group"               2>/dev/null
pkill -9 -f "rviz2"                    2>/dev/null
pkill -9 -f "robot_state_publisher"    2>/dev/null
pkill -9 -f "static_transform_publisher" 2>/dev/null

# 给 OS 时间释放 ROS DDS 共享内存 / FastRTPS socket
sleep 2

echo "[cleanup] residual processes (should be empty):"
ps aux | grep -E "ign gazebo|controller_manager|move_group|rviz2|gz_ros2|robot_state_pub" \
       | grep -v grep || echo "  (none)"

echo "[cleanup] done."

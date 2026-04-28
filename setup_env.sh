#!/bin/bash
# RobotSim 环境配置脚本
# 用法: source ~/RobotSim/setup_env.sh

ROBOTSIM_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# ROS 2 Humble
source /opt/ros/humble/setup.bash

# 工作空间 overlay（如果已经构建）
if [ -f "${ROBOTSIM_DIR}/install/setup.bash" ]; then
    source "${ROBOTSIM_DIR}/install/setup.bash"
fi

# Gazebo 模型资源路径（纯 Alicia-D）
export GZ_SIM_RESOURCE_PATH="${ROBOTSIM_DIR}/alicia_gz_sim/models:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"

# 让 Gazebo 能找到 alicia_d_descriptions 安装后的 share（包含 mesh）
if [ -d "${ROBOTSIM_DIR}/install/alicia_d_descriptions/share" ]; then
    export GZ_SIM_RESOURCE_PATH="${ROBOTSIM_DIR}/install/alicia_d_descriptions/share:${GZ_SIM_RESOURCE_PATH}"
fi

# ----------------------------------------------------------------------
# WSL2 集显软件渲染配置 (Mesa llvmpipe + OGRE1)
#   - 集显在 WSLg 下没有 GPU 直通时回退到 kms_swrast,
#     OGRE2 的 GL3Plus 后端在 mipmap 生成路径上崩溃。
#   - 强制走纯软件渲染 (LIBGL_ALWAYS_SOFTWARE) + Qt 软件后端,
#     配合 bin_scene.sdf 中切到 ogre1 渲染引擎,可在集显环境下稳定运行。
#   - 如果你的机器有可用 GPU 直通 (`/dev/dri/renderD128` 可读),
#     可以把这一段注释掉以获得更好性能。
# ----------------------------------------------------------------------
export LIBGL_ALWAYS_SOFTWARE=1
export OGRE_RTT_MODE=Copy
export QT_QUICK_BACKEND=software
export MESA_GL_VERSION_OVERRIDE=3.3

echo "[RobotSim] Environment configured."
echo "  ROS_DISTRO    = ${ROS_DISTRO}"
echo "  ROBOTSIM_DIR  = ${ROBOTSIM_DIR}"
echo "  GZ_SIM_RESOURCE_PATH = ${GZ_SIM_RESOURCE_PATH}"
echo "  Software rendering: LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}"

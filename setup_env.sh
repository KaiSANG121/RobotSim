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

# 让 Gazebo 能找到 alicia_d_descriptions 安装后的 share（包含 mesh）
if [ -d "${ROBOTSIM_DIR}/install/alicia_d_descriptions/share" ]; then
    export GZ_SIM_RESOURCE_PATH="${ROBOTSIM_DIR}/install/alicia_d_descriptions/share:${GZ_SIM_RESOURCE_PATH}"
fi
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"

# Keep ROS launch/runtime logs inside the workspace so remote/headless tests are
# easy to collect and do not depend on write access to ~/.ros.
export ROS_LOG_DIR="${ROBOTSIM_ROS_LOG_DIR:-${ROBOTSIM_DIR}/log/ros}"

# ----------------------------------------------------------------------
# Rendering profile
#
# ROBOTSIM_RENDERING=auto      默认。WSL2 走软件渲染, 其他 Linux/Jetson 走 GPU。
# ROBOTSIM_RENDERING=software  强制 Mesa llvmpipe + Qt software, 用于 WSL2/虚拟机。
# ROBOTSIM_RENDERING=gpu       取消 WSL2 软件渲染变量, 用于 Jetson/独显机器。
# ----------------------------------------------------------------------
ROBOTSIM_RENDERING="${ROBOTSIM_RENDERING:-auto}"
ROBOTSIM_RENDERING_ACTIVE="${ROBOTSIM_RENDERING}"

if [ "${ROBOTSIM_RENDERING}" = "auto" ]; then
    if grep -qi "microsoft" /proc/version 2>/dev/null; then
        ROBOTSIM_RENDERING_ACTIVE="software"
    else
        ROBOTSIM_RENDERING_ACTIVE="gpu"
    fi
fi

case "${ROBOTSIM_RENDERING_ACTIVE}" in
    software)
        export LIBGL_ALWAYS_SOFTWARE=1
        export OGRE_RTT_MODE=Copy
        export QT_QUICK_BACKEND=software
        export MESA_GL_VERSION_OVERRIDE=3.3
        ;;
    gpu)
        unset LIBGL_ALWAYS_SOFTWARE
        unset OGRE_RTT_MODE
        unset QT_QUICK_BACKEND
        unset MESA_GL_VERSION_OVERRIDE
        ;;
    *)
        echo "[RobotSim] Unknown ROBOTSIM_RENDERING=${ROBOTSIM_RENDERING}; expected auto/software/gpu."
        echo "[RobotSim] Leaving existing rendering environment unchanged."
        ;;
esac

echo "[RobotSim] Environment configured."
echo "  ROS_DISTRO    = ${ROS_DISTRO}"
echo "  ROBOTSIM_DIR  = ${ROBOTSIM_DIR}"
echo "  GZ_SIM_RESOURCE_PATH = ${GZ_SIM_RESOURCE_PATH}"
echo "  ROS_LOG_DIR = ${ROS_LOG_DIR}"
echo "  ROBOTSIM_RENDERING = ${ROBOTSIM_RENDERING_ACTIVE}"
echo "  LIBGL_ALWAYS_SOFTWARE = ${LIBGL_ALWAYS_SOFTWARE:-unset}"

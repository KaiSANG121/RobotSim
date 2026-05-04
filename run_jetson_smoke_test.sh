#!/bin/bash
# Headless health checks for RobotSim on Jetson / Ubuntu 22.04 / ROS 2 Humble.
# Run this after sim_demo.launch.py is already up.

set -uo pipefail

ROBOTSIM_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPORT_FILE="${1:-${ROBOTSIM_DIR}/jetson_smoke_report.md}"

set +u
source "${ROBOTSIM_DIR}/setup_env.sh"
if [ -f "${ROBOTSIM_DIR}/install/setup.bash" ]; then
    source "${ROBOTSIM_DIR}/install/setup.bash"
fi
set -u

run_check() {
    local title="$1"
    shift

    echo
    echo "[smoke] ${title}"
    {
        echo
        echo "## ${title}"
        echo
        echo '```bash'
        "$@"
        echo '```'
    } >> "${REPORT_FILE}"
}

echo "# RobotSim Jetson Smoke Report" > "${REPORT_FILE}"
echo >> "${REPORT_FILE}"
echo "- Date: $(date -Is)" >> "${REPORT_FILE}"
echo "- Host: $(hostname)" >> "${REPORT_FILE}"
echo "- Kernel: $(uname -a)" >> "${REPORT_FILE}"
if [ -r /proc/device-tree/model ]; then
    echo "- Jetson model: $(tr -d '\0' </proc/device-tree/model)" >> "${REPORT_FILE}"
fi
echo "- Rendering: ${ROBOTSIM_RENDERING_ACTIVE:-unknown}" >> "${REPORT_FILE}"

run_check "ROS controllers" ros2 control list_controllers
run_check "joint_states rate" timeout 10 ros2 topic hz /joint_states
run_check "RGB camera rate" timeout 10 ros2 topic hz /rgbd_camera/image
run_check "Depth camera rate" timeout 10 ros2 topic hz /rgbd_camera/depth_image
run_check "world to rgbd_camera_frame TF" timeout 5 ros2 run tf2_ros tf2_echo world rgbd_camera_frame
run_check "world to base_link TF" timeout 5 ros2 run tf2_ros tf2_echo world base_link

if command -v tegrastats >/dev/null 2>&1; then
    run_check "tegrastats snapshot" timeout 3 tegrastats --interval 1000
fi

echo
echo "[smoke] report written to ${REPORT_FILE}"

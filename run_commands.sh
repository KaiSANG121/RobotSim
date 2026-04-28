#!/bin/bash
source ~/RobotSim/setup_env.sh
source ~/RobotSim/install/setup.bash

echo "# 1. 控制器都 active" > terminal_output.md
echo "\`\`\`bash" >> terminal_output.md
ros2 control list_controllers >> terminal_output.md
echo "\`\`\`" >> terminal_output.md

echo "# 2. 关节状态在发" >> terminal_output.md
echo "\`\`\`bash" >> terminal_output.md
timeout 10 ros2 topic hz /joint_states >> terminal_output.md
echo "\`\`\`" >> terminal_output.md

echo "# 3. RGBD 相机数据" >> terminal_output.md
echo "## /rgbd_camera/image" >> terminal_output.md
echo "\`\`\`bash" >> terminal_output.md
timeout 10 ros2 topic hz /rgbd_camera/image >> terminal_output.md
echo "\`\`\`" >> terminal_output.md

echo "## /rgbd_camera/depth_image" >> terminal_output.md
echo "\`\`\`bash" >> terminal_output.md
timeout 10 ros2 topic hz /rgbd_camera/depth_image >> terminal_output.md
echo "\`\`\`" >> terminal_output.md

echo "# 4. TF" >> terminal_output.md
echo "## world to rgbd_camera_frame" >> terminal_output.md
echo "\`\`\`bash" >> terminal_output.md
timeout 5 ros2 run tf2_ros tf2_echo world rgbd_camera_frame >> terminal_output.md
echo "\`\`\`" >> terminal_output.md

echo "## world to base_link" >> terminal_output.md
echo "\`\`\`bash" >> terminal_output.md
timeout 5 ros2 run tf2_ros tf2_echo world base_link >> terminal_output.md
echo "\`\`\`" >> terminal_output.md
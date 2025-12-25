#!/bin/bash
# 水平面蛇形机器人可视化启动脚本

echo "=========================================="
echo "启动水平面蛇形机器人可视化"
echo "=========================================="
echo ""
echo "这个脚本会启动："
echo "  1. RViz2 可视化"
echo "  2. 验证脚本（发布测试命令）"
echo ""
echo "按 Ctrl+C 停止"
echo "=========================================="
echo ""

# Source 环境
source install/setup.bash

# 启动 RViz 和 robot_state_publisher
ros2 launch asr_sdm_description horizontal_snake_visualization.launch.py &
LAUNCH_PID=$!

# 等待 RViz 启动
sleep 3

# 启动验证脚本
python3 src/asr_sdm_universe/control/asr_sdm_controller/verify_head_tracking_rviz.py

# 清理
kill $LAUNCH_PID

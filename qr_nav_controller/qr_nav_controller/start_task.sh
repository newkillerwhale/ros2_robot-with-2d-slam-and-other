#!/bin/bash
# 获取工作空间路径
WORKSPACE_DIR="/home/zp/ros_car/ros_car"
source $WORKSPACE_DIR/install/setup.bash

echo "正在后台启动任务系统..."

# 使用 & 让指令在后台运行，并记录它们的 PID
ros2 launch sllidar_ros2 sllidar_a1_launch.py > /dev/null 2>&1 &
sleep 2
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py > /dev/null 2>&1 &
sleep 2
ros2 launch yahboomcar_nav navigation_teb_launch.py > /dev/null 2>&1 &
sleep 3
#ros2 run qr_nav_controller task_manager > /dev/null 2>&1 &

# 关键：保持脚本运行，这样 all_manager 杀这个脚本时，能顺带杀掉这一组
wait

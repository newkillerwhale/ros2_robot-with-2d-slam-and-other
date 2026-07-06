#!/bin/bash
# 获取工作空间路径
WORKSPACE_DIR="/home/zp/ros_car/ros_car"
source $WORKSPACE_DIR/install/setup.bash

echo "正在后台启动建图系统..."

# 使用 & 让指令在后台运行，并记录它们的 PID
ros2 run yahboomcar_bringup Mcnamu_driver_X3 > /dev/null 2>&1 &
sleep 2
ros2 launch yahboomcar_nav map_gmapping_a1_launch.py> /dev/null 2>&1 &
sleep 2
ros2 launch yahboomcar_nav ekf_x1_x3_launch.py> /dev/null 2>&1 &
sleep 2

# 关键：保持脚本运行，这样 all_manager 杀这个脚本时，能顺带杀掉这一组
wait

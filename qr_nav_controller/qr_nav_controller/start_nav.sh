#!/bin/bash
# ==========================================
# 导航一键启动脚本 - 支持地图传参
# ==========================================

# 获取工作空间路径
WORKSPACE_DIR="/home/zp/ros_car/ros_car"
source $WORKSPACE_DIR/install/setup.bash

# 接收从 Python 传过来的第一个参数作为地图名
# 如果 App 没传参数，默认使用 "yahboomcar"
MAP_NAME=${1:-"yahboomcar"}

# 拼接地图 YAML 的绝对路径
MAP_PATH="$WORKSPACE_DIR/src/yahboomcar_nav/maps/$MAP_NAME.yaml"

echo ">>> 正在后台启动导航系统..."
echo ">>> 使用地图路径: $MAP_PATH"

# 1. 启动雷达
ros2 launch sllidar_ros2 sllidar_a1_launch.py > /dev/null 2>&1 &
sleep 2

# 2. 启动小车底盘驱动
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py > /dev/null 2>&1 &
sleep 2

# 3. 启动导航核心 (传入 map 参数)
# 这里确保参数名是 map，对应你 Launch 文件里的 DeclareLaunchArgument('map', ...)
ros2 launch yahboomcar_nav navigation_teb_launch.py map:=$MAP_PATH > /dev/null 2>&1 &

# 保持脚本运行，等待 all_manager 的关闭信号
wait

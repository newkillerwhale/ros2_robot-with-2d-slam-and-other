from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的路径
    pkg_dir = get_package_share_directory('dm_imu')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'imu.rviz')

    return LaunchDescription([
        # 启动 IMU 节点
        Node(
            package='dm_imu',
            executable='imu_node',
            name='dm_imu_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM1',  # 参数 port
                'baud': 921600          # 参数 baud
            }]
        ),

        # 启动 RViz
        Node(
            package='rviz2',  # ROS2 中使用 rviz2 而不是 rviz
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_path],  # 指定 RViz 配置文件
            output='screen'
        )
    ])
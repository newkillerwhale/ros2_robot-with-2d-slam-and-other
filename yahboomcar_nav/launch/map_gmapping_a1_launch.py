from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    laser_bringup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
         '/laser_bringup_launch.py'])
    )

    # 方案 1: 使用 slam_toolbox (推荐)
    slam_toolbox_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('slam_toolbox'), 'launch'),
         '/online_async_launch.py'])  # 或 online_sync_launch.py
    )

    # 方案 2: 直接启动 slam_toolbox 节点（更灵活）
    # slam_toolbox_node = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': False,
    #         'max_laser_range': 12.0,
    #         'resolution': 0.05,
    #     }],
    #     remappings=[
    #         ('/scan', '/scan'),
    #         ('/odom', '/odom'),
    #     ]
    # )

    return LaunchDescription([
        laser_bringup_launch, 
        slam_toolbox_launch,  # 或 slam_toolbox_node
    ])

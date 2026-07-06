from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM1',
            description='Serial port for IMU'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='921600',
            description='Baudrate for IMU'
        ),

        # 启动节点
        Node(
            package='dm_imu',
            executable='imu_node',
            name='dm_imu_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud')
            }]
        )
    ])
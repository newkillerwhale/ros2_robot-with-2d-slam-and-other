#include "imu_driver.h"
#include <iostream>
#include <thread>
#include <condition_variable>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 创建节点选项并启用自动参数声明
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    // 创建 IMU 接口实例
    auto imuInterface = std::make_shared<dmbot_serial::DmImu>(options);
    rclcpp::Rate loop_rate(1000); // 1000 Hz

    // 主循环
    while (rclcpp::ok())
    {
        loop_rate.sleep();
    }

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}





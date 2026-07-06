#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <string>
#include <cmath>

using std::placeholders::_1;

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher()
        : Node("base_node")
    {
        // 声明参数
        this->declare_parameter<double>("wheelbase", 0.25);
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
        this->declare_parameter<double>("linear_scale_x", 1.0);
        this->declare_parameter<double>("linear_scale_y", 1.0);
        this->declare_parameter<bool>("pub_odom_tf", false);

        // 获取参数
        this->get_parameter<double>("linear_scale_x", linear_scale_x_);
        this->get_parameter<double>("linear_scale_y", linear_scale_y_);
        this->get_parameter<double>("wheelbase", wheelbase_);
        this->get_parameter<bool>("pub_odom_tf", pub_odom_tf_);
        this->get_parameter<std::string>("odom_frame", odom_frame);
        this->get_parameter<std::string>("base_footprint_frame", base_footprint_frame);

        // 初始化 TF 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 订阅和发布者 (队列大小建议设为 10，过大的队列会增加延迟)
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "vel_raw", 10, std::bind(&OdomPublisher::handle_vel, this, _1));
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_raw", 10);

        // 初始化时间
        last_vel_time_ = this->get_clock()->now();
    }

private:
    void handle_vel(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
    {
        // 1. 统一获取当前节点时钟时间（这是解决外推错误的关键）[cite: 1]
        rclcpp::Time current_time = this->get_clock()->now();

        // 2. 首次运行处理，防止 vel_dt_ 异常[cite: 1]
        if (first_time_) {
            last_vel_time_ = current_time;
            first_time_ = false;
            return;
        }

        // 3. 计算时间差 (dt)[cite: 1]
        double vel_dt = (current_time - last_vel_time_).seconds();
        last_vel_time_ = current_time;

        // 如果 dt 异常大（如超过 1s），可能是系统卡顿，建议跳过本次计算[cite: 1]
        if (vel_dt > 1.0 || vel_dt <= 0.0) return;

        // 获取速度[cite: 1]
        double linear_velocity_x = msg->linear.x * linear_scale_x_;
        double linear_velocity_y = msg->linear.y * linear_scale_y_;
        double angular_velocity_z = msg->angular.z;

        // 4. 积分计算位置和航向角[cite: 1]
        double delta_heading = angular_velocity_z * vel_dt;
        double delta_x = (linear_velocity_x * cos(heading_) - linear_velocity_y * sin(heading_)) * vel_dt;
        double delta_y = (linear_velocity_x * sin(heading_) + linear_velocity_y * cos(heading_)) * vel_dt;

        x_pos_ += delta_x;
        y_pos_ += delta_y;
        heading_ += delta_heading;

        // 5. 处理四元数[cite: 1]
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0.00, 0.00, heading_);

        // 6. 发布 Odometry 消息[cite: 1]
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time; // 使用统一的时间戳[cite: 1]
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = base_footprint_frame;

        odom.pose.pose.position.x = x_pos_;
        odom.pose.pose.position.y = y_pos_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = myQuaternion.x();
        odom.pose.pose.orientation.y = myQuaternion.y();
        odom.pose.pose.orientation.z = myQuaternion.z();
        odom.pose.pose.orientation.w = myQuaternion.w();

        // 填充协方差（Nav2 需要这些值来判断里程计可靠性）[cite: 1]
        odom.pose.covariance.fill(0.0);
        odom.pose.covariance[0] = 0.001;  // x
        odom.pose.covariance[7] = 0.001;  // y
        odom.pose.covariance[35] = 0.001; // yaw

        odom.twist.twist.linear.x = linear_velocity_x;
        odom.twist.twist.linear.y = linear_velocity_y;
        odom.twist.twist.angular.z = angular_velocity_z;
        odom.twist.covariance.fill(0.0);
        odom.twist.covariance[0] = 0.0001;
        odom.twist.covariance[35] = 0.0001;

        odom_publisher_->publish(odom);

        // 7. 发布 TF 变换[cite: 1]
        if (pub_odom_tf_)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = current_time; // 必须与 odom 消息时间戳严丝合缝[cite: 1]
            t.header.frame_id = odom_frame;
            t.child_frame_id = base_footprint_frame;

            t.transform.translation.x = x_pos_;
            t.transform.translation.y = y_pos_;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = myQuaternion.x();
            t.transform.rotation.y = myQuaternion.y();
            t.transform.rotation.z = myQuaternion.z();
            t.transform.rotation.w = myQuaternion.w();

            tf_broadcaster_->sendTransform(t);
        }
    }

    // 成员变量
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double linear_scale_x_ = 1.0;
    double linear_scale_y_ = 1.0;
    double x_pos_ = 0.0;
    double y_pos_ = 0.0;
    double heading_ = 0.0;
    double wheelbase_ = 0.25;
    bool pub_odom_tf_ = false;
    bool first_time_ = true;
    
    rclcpp::Time last_vel_time_;
    std::string odom_frame = "odom";
    std::string base_footprint_frame = "base_footprint";
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}

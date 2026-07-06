#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

class RobotManager(Node):
    def __init__(self):
        # 初始化 ROS2 基础节点（主要用于提供系统时钟和日志打印）
        super().__init__('robot_manager')
        
        # 1. 初始化 Nav2 导航控制核心
        self.nav = BasicNavigator()
        
        # 2. 从 RViz 日志中提取并清洗出的 12 个顺序目标点数据 [x, y, z_quat, w_quat]
        self.waypoint_list = [
            [0.577364, 0.0303617, 0.0289215, 0.999582],   # P1
            [1.11061, 0.0419989, -0.63543, 0.772159],     # P2
            [1.23693, -0.55939, -0.179482, 0.983761],     # P3
            [1.62779, -0.580917, 0.390942, 0.920415],     # P4
            [1.7324, 0.0374521, 0.976355, 0.216173],      # P5
            [1.03013, 0.272374, 0.752633, 0.65844],       # P6
            [1.18174, 0.791343, 0.175505, 0.984479],      # P7
            [1.76274, 0.449419, -0.523705, 0.851899],     # P8
            [1.94688, 0.11959, 0.0402934, 0.999188],      # P9
            [2.51021, 0.1545, 0.0374193, 0.9993],         # P10
            [2.58639, 0.1927, 0.0415961, 0.999135],       # P11
            [2.74039, 0.224071, 0.0438426, 0.999038]      # P12 终点
        ]

    def create_pose(self, coords):
        """将提取的数组包装为 Nav2 接收的 PoseStamped 消息"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = coords[0]
        pose.pose.position.y = coords[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = coords[2]
        pose.pose.orientation.w = coords[3]
        return pose

    def run_patrol_flow(self):
        """核心无干扰执行逻辑：启动后自动顺序遍历所有点"""
        
        # 1. 确保导航堆栈完全启动就绪
        self.get_logger().info("正在等待 Nav2 导航生命周期服务器就绪...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("Nav2 状态正常，开始全自动巡航作业！")
        
        total_points = len(self.waypoint_list)
        
        # 2. 依次遍历并导航前往每一个目标点
        for index, coords in enumerate(self.waypoint_list, start=1):
            self.get_logger().info(f"==> 任务目标更新 [{index}/{total_points}]: 正在奔赴坐标点 ({coords[0]}, {coords[1]})")
            
            # 下发导航点
            target_pose = self.create_pose(coords)
            self.nav.goToPose(target_pose)
            
            # 3. 阻塞等待当前点到站
            while not self.nav.isTaskComplete():
                # 纯净版不执行任何计算，仅维持微小的查询频率，避免挤占 CPU 算力
                time.sleep(0.1)

            # 4. 判断本点结果
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"成功到达第 [{index}] 个点！在原地休整 1 秒...")
                time.sleep(1.0) # 在每个点驻留 1 秒鐘，用于微调对齐车头，或可修改为 0 
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f"警告：第 [{index}] 个点的任务中途被手动取消。")
                break
            elif result == TaskResult.FAILED:
                self.get_logger().error(f"错误：前往第 [{index}] 个点时宣告失败！放弃后续巡航。")
                break

        # 5. 走完全部流程
        self.get_logger().info("★ 所有给定的多点巡航任务已全部结束。")

def main():
    rclpy.init()
    node = RobotManager()
    
    try:
        # 由于不需要后台常驻订阅，此处直接调用主循环方法，不使用 rclpy.spin()
        node.run_patrol_flow()
    except KeyboardInterrupt:
        node.get_logger().warn("用户强行中断了巡航程序。")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

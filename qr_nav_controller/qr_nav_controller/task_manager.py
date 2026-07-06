import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String  # 用于与 Flutter 通信
import cv2
import cv2.aruco as aruco
import time
import math
import json

class QRTaskSystem(Node): # 继承 Node
    def __init__(self):
        super().__init__('qr_task_manager')
        
        # 1. 初始化导航器
        self.nav = BasicNavigator()
        
        # 2. 与 Flutter 通信的接口
        # 订阅来自 Flutter 的指令 (例如：{"cmd": "start"})
        self.cmd_sub = self.create_subscription(String, '/app/commands', self.app_callback, 10)
        # 向 Flutter 发布实时状态 (例如：{"state": "前往A点", "progress": 0.2})
        self.status_pub = self.create_publisher(String, '/app/task_status', 10)
        
        # ArUco 配置
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        
        # 预设坐标点 (x, y, z_quat, w_quat)
        self.location_map = {
            "point_A": [1.46964, -0.224464, -0.968794, 0.247867],
            "1": [2.64801, -0.3273, -0.556327, 0.830963],
            "2": [3.42073, 0.0903592, -0.368623, 0.929579],
            "3": [3.89307, 0.902398, 0.270213, 0.962801]
        }

    def publish_status(self, state, progress):
        """向 Flutter 发送 JSON 格式的状态反馈"""
        msg = String()
        msg.data = json.dumps({"state": state, "progress": progress})
        self.status_pub.publish(msg)
        self.get_logger().info(f"状态更新: {state} ({int(progress*100)}%)")

    def create_pose(self, coords):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = coords[0]
        pose.pose.position.y = coords[1]
        # 直接使用你从 RViz 获取的四元数
        pose.pose.orientation.z = coords[2]
        pose.pose.orientation.w = coords[3]
        return pose

    def get_qr_id_direct(self, device_id=0):
        cap = cv2.VideoCapture(device_id)
        if not cap.isOpened(): return None
        for _ in range(10): cap.grab()
        ret, frame = cap.read()
        cap.release()
        if not ret: return None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        return str(ids[0][0]) if ids is not None else None

    def app_callback(self, msg):
        """接收 Flutter 的启动指令"""
        try:
            data = json.loads(msg.data)
            if data.get('cmd') == 'start':
                self.get_logger().info("收到 Flutter 启动指令，开始全流程作业...")
                self.run_task_flow()
        except Exception as e:
            self.get_logger().error(f"解析指令失败: {e}")

    def run_task_flow(self):
        """全流程自动化逻辑，带有状态反馈"""
        
        # 1. 前往 A 点
        self.publish_status("正在前往 A 点识别区", 0.1)
        self.nav.goToPose(self.create_pose(self.location_map["point_A"]))
        
        while not self.nav.isTaskComplete():
            # 可以在这里根据距离动态计算进度发布给 Flutter
            pass

        if self.nav.getResult() != TaskResult.SUCCEEDED:
            self.publish_status("错误：前往 A 点失败", 0.0)
            return

        # 2. 识别二维码
        self.publish_status("到达 A 点，正在扫描二维码", 0.4)
        time.sleep(1.5)
        qr_id = self.get_qr_id_direct(0)
        
        if not qr_id:
            self.publish_status("错误：未识别到二维码", 0.4)
            return
        
        self.publish_status(f"识别成功：ID {qr_id}", 0.6)

        # 3. 导航到对应目标
        if qr_id in self.location_map:
            self.publish_status(f"正在前往目标点 {qr_id}", 0.7)
            self.nav.goToPose(self.create_pose(self.location_map[qr_id]))
            
            while not self.nav.isTaskComplete():
                pass
            
            if self.nav.getResult() == TaskResult.SUCCEEDED:
                self.publish_status("任务圆满完成", 1.0)
            else:
                self.publish_status("错误：前往目标点失败", 0.7)
        else:
            self.publish_status(f"错误：库中无 ID {qr_id}", 0.6)

def main():
    rclpy.init()
    node = QRTaskSystem()
    # 使用 spin 让节点持续运行，等待 Flutter 的指令
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

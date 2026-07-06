import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import cv2
import cv2.aruco as aruco
import time
import math

class QRTaskSystem:
    def __init__(self):
        # 初始化导航器
        self.nav = BasicNavigator()
        
        # ArUco 字典配置 (使用常用的 4x4_50)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        
        # 预设坐标点 (x, y, theta_in_degrees)
        self.location_map = {
            "point_A": [0.164065, 0.859039, 0.0685463, 0.997648],
            "1": [0.894382, 1.65384, 0.00363711, 0.999993],
            "2": [1.04005, 2.82406, 0.0200755, 0.999798],
            "3": [0.125935, 4.00762, 0.779063, 0.626945]
        }

    def create_pose(self, coords):
        """将 [x, y, theta] 转换为 PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = coords[0]
        pose.pose.position.y = coords[1]
        
        # 角度转四元数 (简单 Z 轴旋转)
        half_rad = math.radians(coords[2]) / 2.0
        pose.pose.orientation.z = math.sin(half_rad)
        pose.pose.orientation.w = math.cos(half_rad)
        return pose

    def get_qr_id_direct(self, device_id=0):
        """直接调用硬件读取二维码"""
        cap = cv2.VideoCapture(device_id)
        if not cap.isOpened():
            return None
        
        # 跳过前 10 帧让相机自动曝光
        for _ in range(10):
            cap.grab()
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret: return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            return str(ids[0][0])
        return None

    def run(self):
        # 1. 导航到 A 点
        print(">>> 正在前往 A 点...")
        self.nav.goToPose(self.create_pose(self.location_map["point_A"]))
        
        while not self.nav.isTaskComplete():
            pass

        if self.nav.getResult() != TaskResult.SUCCEEDED:
            print("!!! 导航至 A 点失败")
            return

        # 2. 识别二维码
        print(">>> 到达 A 点，开始扫描...")
        time.sleep(1.5) # 停稳避免晃动
        qr_id = self.get_qr_id_direct(0)
        
        if not qr_id:
            print("!!! 未识别到二维码")
            return
        
        print(f">>> 识别成功！目标 ID: {qr_id}")

        # 3. 导航到对应目标
        if qr_id in self.location_map:
            print(f">>> 正在前往目标点 {qr_id}...")
            self.nav.goToPose(self.create_pose(self.location_map[qr_id]))
            
            while not self.nav.isTaskComplete():
                pass
            
            if self.nav.getResult() == TaskResult.SUCCEEDED:
                print(">>> 任务圆满完成！")
        else:
            print(f"!!! 坐标库中未定义 ID: {qr_id}")

def main():
    rclpy.init()
    system = QRTaskSystem()
    system.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

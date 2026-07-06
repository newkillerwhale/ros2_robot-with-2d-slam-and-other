#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 机器人全自动导航与动态决策节点 - YOLOv11本地识别 + 12点线上OCR首位插队版
1. 9/10 点利用 OpenVINO (YOLOv11) 确定 3 个物体自右向左的物理槽位顺序并关联坐标
2. 12 点原地等待 3 秒后触发线上 OCR，识别到的关键词作为“首发目标”
3. 动态规划后续前往点，且到站后完美触发对应的正确语音播报
"""

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import cv2
import numpy as np
import openvino as ov
import os
import sys
import requests
import base64

# ==================== 1. 视觉识别、线上OCR与音频配置 ====================
MODEL_PATH = '/home/zp/ros_car/zpzp_openvino_model/zpzp.xml'
DEVICE     = 'GPU'
CONF_THRES = 0.35
NMS_THRES  = 0.45
QUEUE_SZ   = 2

# 🔑 请在此处填写你在百度智能云申请的 OCR 密钥
API_KEY = "9oH224LLu4tRAcreR1kmELQC"
SECRET_KEY = "jasB9WO9m3g6LKVP7kHflw8qeJVG4OHe"

# 动态获取当前脚本所在的绝对路径，确保同目录下的 mp3 能被精准定位
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

AUDIO_MAPPING = {
    'cube': '这个是正方体.mp3',
    'sphere': '这个是球体.mp3',
    'cylinder': '圆柱体.mp3'
}

# OCR 线上文本到 YOLO 标签名的映射字典
OCR_TO_YOLO_MAPPING = {
    "正方体": "cube",
    "球体": "sphere",
    "圆柱体": "cylinder"
}

# 目标类别映射（基于你的 YOLO 模型的实际类别顺序）
CLASS_NAMES = {0: 'cube', 1: 'sphere', 2: 'cylinder'}

# ==================== 2. 整合后的机器人控制节点 ====================
class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        
        # 初始化 Nav2 导航控制核心
        self.nav = BasicNavigator()
        
        # 前 12 个巡航目标点（到 P12 触发线上决策识别）
        self.waypoint_list = [
            [0.577364, 0.0303617, 0.0289215, 0.999582],   # P1
            [1.11061, 0.0419989, -0.63543, 0.772159],     # P2
            [1.23693, -0.55939, -0.179482, 0.983761],     # P3
            [1.62779, -0.580917, 0.390942, 0.920415],     # P4
            [1.7324, 0.0374521, 0.976355, 0.216173],      # P5
            [1.03013, 0.272374, 0.752633, 0.65844],       # P6
            [1.18174, 0.791343, 0.175505, 0.984479],      # P7
            [1.76274, 0.449419, -0.523705, 0.851899],     # P8
            [1.94688, 0.11959, 0.0402934, 0.999188],      # P9  -> 触发 YOLO 本地识别（静默）
            [2.51021, 0.1545, 0.0374193, 0.9993],          # P10 -> 触发 YOLO 本地识别并锁定最终顺序（静默）
            [2.58639, 0.1927, 0.0415961, 0.999135],       # P11
            [2.74039, 0.224071, 0.0438426, 0.999038],      # P12 -> 线上决策决策点
        ]
        
        # 最后的三个固定释放点物理坐标池（按空间原本自右向左排列绑定）
        self.raw_release_poses = [
            [3.13837, -0.430822, 0.714831, 0.699297],     # 物理位置 1 (原本的 P13)
            [3.09834, 0.214466, 0.732353, 0.680925],      # 物理位置 2 (原本的 P14)
            [3.01064, 0.794791, 0.731985, 0.681321]       # 物理位置 3 (原本的 P15)
        ]
        
        # 用于记忆 YOLO 在 9/10 号点识别到的【从右到左】的物体标签清单 (例如: ['cube', 'sphere', 'cylinder'])
        self.right_to_left_objects = []
        
        # 初始化 OpenVINO 环境
        self.init_openvino()
        
        # 建立线上 OCR 访问凭证
        self.ocr_token = self.get_access_token()
        if not self.ocr_token:
            self.get_logger().error("❌ 线上 OCR Token 获取失败！请检查互联网连接及密钥配置！")
            sys.exit(1)
        self.get_logger().info("✔ 线上高精度 OCR 连通就绪。")

    def init_openvino(self):
        """初始化 OpenVINO 模型与推理双缓冲队列（完好保留原有逻辑）"""
        self.get_logger().info("正在初始化 OpenVINO 引擎...")
        self.core = ov.Core()
        self.model = self.core.read_model(MODEL_PATH)
        self.compiled = self.core.compile_model(self.model, DEVICE, {'PERFORMANCE_HINT': 'THROUGHPUT'})
        self.H, self.W = int(self.compiled.input(0).shape[2]), int(self.compiled.input(0).shape[3])
        
        self.req_queue = ov.AsyncInferQueue(self.compiled, jobs=QUEUE_SZ)
        self.in_tensors = [ov.Tensor(self.compiled.input(0).get_element_type(), [1, 3, self.H, self.W]) for _ in range(QUEUE_SZ)]
        self.out_tensors = [ov.Tensor(self.compiled.output(0).get_element_type(), self.compiled.output(0).shape) for _ in range(QUEUE_SZ)]
        
        for i in range(QUEUE_SZ):
            self.req_queue[i].set_input_tensor(self.in_tensors[i])
            self.req_queue[i].set_output_tensor(self.out_tensors[i])

    def get_access_token(self):
        """获取百度鉴权 Token"""
        url = f"https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id={API_KEY}&client_secret={SECRET_KEY}"
        headers = {'Content-Type': 'application/json', 'Accept': 'application/json'}
        try:
            response = requests.post(url, headers=headers, timeout=5)
            if response.status_code == 200:
                return response.json().get("access_token")
        except Exception as e:
            self.get_logger().error(f"OCR Token 获取错误: {e}")
        return None

    def create_pose(self, coords):
        """包装坐标点为 Nav2 接收的 PoseStamped 消息"""
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

    def postprocess(self, pred, ih, iw):
        """YOLOv11 后处理（完好保留）"""
        pred = pred[0].astype(np.float32)
        boxes, scores, classes = [], [], []
        for i in range(pred.shape[1]):
            row = pred[:, i]
            score = row[4:].max()
            if score < CONF_THRES: continue
            x, y, w, h = row[:4]
            x1 = int((x - w/2) * iw / self.W)
            y1 = int((y - h/2) * ih / self.H)
            x2 = int((x + w/2) * iw / self.W)
            y2 = int((y + h/2) * ih / self.H)
            boxes.append([x1, y1, x2, y2])
            scores.append(float(score))
            classes.append(int(row[4:].argmax()))
            
        idx = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRES, NMS_THRES)
        if len(idx) == 0: return []
        idx = idx[0] if isinstance(idx, tuple) else idx
        return [(boxes[int(i)], classes[int(i)], scores[int(i)]) for i in idx]

    def play_voice(self, name):
        """统一音频播放接口"""
        audio_path = os.path.join(CURRENT_DIR, AUDIO_MAPPING[name])
        if os.path.exists(audio_path):
            self.get_logger().info(f"🔊 正在播放音频文件: {audio_path}")
            os.system(f"play -q {audio_path} &> /dev/null")
        else:
            self.get_logger().error(f"❌ 无法找到目标音频文件: {audio_path}")

    def run_vision_and_save(self, station_idx):
        """在 9, 10 点位运行的本地 YOLO 静默视觉识别与排序记忆（完好保留）"""
        self.get_logger().info(f"=== [点位 {station_idx}] 启动本地 YOLOv11 静默识别 ===")
        
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened(): cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        for _ in range(5): cap.read()  # 预热

        detected_objects = []
        frame_cnt = 0
        t_start = time.time()
        
        while time.time() - t_start < 2.0:
            ret, frame = cap.read()
            if not ret: break
            
            ih, iw = frame.shape[:2]
            blob = cv2.dnn.blobFromImage(frame, 1/255.0, (self.W, self.H), swapRB=True)
            self.in_tensors[frame_cnt % QUEUE_SZ].data[:] = blob
            self.req_queue.start_async()
            frame_cnt += 1
            
            if self.req_queue.is_ready():
                preds = self.out_tensors[(frame_cnt - 1) % QUEUE_SZ].data
                dets = self.postprocess(preds, ih, iw)
                
                for (x1, y1, x2, y2), cls_id, sc in dets:
                    cls_name = CLASS_NAMES.get(cls_id, None)
                    if cls_name in AUDIO_MAPPING:
                        x_center = (x1 + x2) / 2
                        detected_objects.append((x_center, cls_name))

        cap.release()
        self.req_queue.wait_all()
        
        if not detected_objects:
            self.get_logger().warn(f"[点位 {station_idx}] 未识别到有效 YOLO 标靶。")
            return

        unique_targets = {}
        for x_center, name in detected_objects:
            if name not in unique_targets:
                unique_targets[name] = []
            unique_targets[name].append(x_center)
            
        # 按 X 坐标从大到小（自右向左）排序
        right_to_left = sorted(
            [(np.mean(xs), name) for name, xs in unique_targets.items()],
            key=lambda item: item[0],
            reverse=True
        )
        self.right_to_left_objects = [item[1] for item in right_to_left]
        self.get_logger().info(f"✨ [YOLO 记忆成功] 绑定物理从右到左序列：{self.right_to_left_objects}。")

    def execute_online_ocr_decision(self):
        """到达 P12 处先等待 3 秒再识别，捕获插队主目标并映射 YOLO 物理槽位"""
        self.get_logger().info("⏱ [P12 停顿机制] 正在原地静止等待 3 秒钟进行稳定拍照...")
        time.sleep(3.0)  # 🛠️ 满足需求：在点 12 处多等待 3 秒，不要一下进下一步
        
        self.get_logger().info("📸 停顿结束，正在抓取图像并请求云端 OCR 识别...")
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened(): cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        for _ in range(5): cap.read() # 预热消除阴影
        ret, frame = cap.read()
        cap.release()
        
        # 保底队列构建：如果没拿到 YOLO 或 OCR 异常，采用 1->2->3 的原本默认跑图
        fallback_queue = []
        for i in range(min(len(self.right_to_left_objects), len(self.raw_release_poses))):
            fallback_queue.append([self.right_to_left_objects[i], self.raw_release_poses[i]])
            
        if not ret:
            self.get_logger().error("❌ 画面抓取失败，启动默认保底跟随。")
            return fallback_queue

        # 转 base64
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        img_base64 = base64.b64encode(buffer)

        url = f"https://aip.baidubce.com/rest/2.0/ocr/v1/accurate?access_token={self.ocr_token}"
        headers = {'content-type': 'application/x-www-form-urlencoded'}
        payload = {'image': img_base64}
        
        first_match_yolo_name = None
        try:
            response = requests.post(url, data=payload, headers=headers, timeout=6)
            if response.status_code == 200:
                res_json = response.json()
                if "words_result" in res_json:
                    for item in res_json["words_result"]:
                        text = item["words"].strip()
                        
                        # 检测返回的汉字属于哪个 YOLO 对应的类
                        for ch_word, yolo_label in OCR_TO_YOLO_MAPPING.items():
                            if ch_word in text:
                                first_match_yolo_name = yolo_label
                                break
                        if first_match_yolo_name:
                            break
        except Exception as e:
            self.get_logger().error(f"云端 OCR 请求异常: {e}")

        # 如果没有识别到有效核心汉字或者 9/10 号点并未建立起 YOLO 记忆，触发盲跑
        if not first_match_yolo_name or not self.right_to_left_objects:
            self.get_logger().warn("⚠ 未匹配到插队目标，恢复默认顺序投递。")
            return fallback_queue

        self.get_logger().info(f"🎯 [OCR 判定成功] 插队主目标为中文汉字映射的 YOLO 标签: 【{first_match_yolo_name}】")

        # ----------------- 建立物理插队映射逻辑 -----------------
        # 根据 9/10 点存入的从右到左顺序，把每个物体和对应的固定物理槽位坐标完全连线起来
        yolo_to_pose_map = {}
        for idx, name in enumerate(self.right_to_left_objects):
            if idx < len(self.raw_release_poses):
                yolo_to_pose_map[name] = self.raw_release_poses[idx]

        # 如果主目标不在 9/10 点辨识出的池子里，防止死机，加入保底机制
        if first_match_yolo_name not in yolo_to_pose_map:
            self.get_logger().error(f"❌ 致命错误：OCR 识别到的【{first_match_yolo_name}】在本地 YOLO 记忆池中未检索到！启动盲跑。")
            return fallback_queue

        dynamic_tasks = []
        
        # 1. 强制把主目标及其绑定的物理槽位坐标加入队列首位
        dynamic_tasks.append([first_match_yolo_name, yolo_to_pose_map[first_match_yolo_name]])
        
        # 2. 剩余的两个目标无序补齐在屁股后面
        for name in self.right_to_left_objects:
            if name != first_match_yolo_name:
                dynamic_tasks.append([name, yolo_to_pose_map[name]])

        return dynamic_tasks

    def run_patrol_flow(self):
        """全自动控制流"""
        self.get_logger().info("正在等待 Nav2 导航服务就绪...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("开始固定巡航（P1 - P12）...")
        
        # 1. 前置 12 个物理点位导航
        for index, coords in enumerate(self.waypoint_list, start=1):
            self.get_logger().info(f"==> 前进固定点 [{index}/12]: ({coords[0]}, {coords[1]})")
            target_pose = self.create_pose(coords)
            self.nav.goToPose(target_pose)
            
            while not self.nav.isTaskComplete():
                time.sleep(0.1)

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"成功抵达点位 [{index}]。")
                
                # 情景 A：在 9 和 10 号点位进行本地 YOLOv11 图像处理和空间自右向左排序记忆
                if index in [9, 10]:
                    self.run_vision_and_save(station_idx=index)
                    time.sleep(1.0)
                else:
                    time.sleep(1.0)
            else:
                self.get_logger().error(f"❌ 奔赴固定点 [{index}] 宣告失败，导航强行熔断。")
                return

        # 2. 到达 P12，触发【原地等待3秒】并调用 OCR 决策插队
        dynamic_delivery_tasks = self.execute_online_ocr_decision()

        route_str = " -> ".join([item[0] for item in dynamic_delivery_tasks])
        self.get_logger().info(f"🚀 [航线规划完成] 最终确定的多释放点巡航顺序：{route_str}")

        # 3. 按照组装结果依次奔赴最终目标，并完美触发各自对应的语音播报
        for idx, (target_name, coords) in enumerate(dynamic_delivery_tasks, start=1):
            self.get_logger().info(f"🎯 决策投递 [{idx}/3]: 正在奔赴物体【{target_name}】对应的物理释放点坐标 ({coords[0]}, {coords[1]})")
            
            target_pose = self.create_pose(coords)
            self.nav.goToPose(target_pose)
            
            while not self.nav.isTaskComplete():
                time.sleep(0.1)

            if self.nav.getResult() == TaskResult.SUCCEEDED:
                # 🛠️ 满足需求：在这里触发原本完好无损的语音播报逻辑，通过标签直接精确抓取正确的 mp3
                self.get_logger().info(f"✔ 成功安全到达目标位置 ──> 正在进行【{target_name}】专属音频文件播报")
                self.play_voice(target_name)
                time.sleep(2.0)  # 留出充足播放间隙
            else:
                self.get_logger().error(f"❌ 动态前往【{target_name}】对应的点位失败。")

        self.get_logger().info("★ 恭喜！YOLO排序、12点时停OCR优先插队、及对应释放点正确播报任务全部顺利完成！")

# ==================== 3. 主函数程序入口 ====================
def main():
    rclpy.init()
    node = RobotManager()
    try:
        node.run_patrol_flow()
    except KeyboardInterrupt:
        node.get_logger().warn("用户强行中断了巡航程序。")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

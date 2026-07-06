#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import cv2
import numpy as np
import openvino as ov
import os

# ==================== 1. 视觉识别与音频配置 ====================
MODEL_PATH = '/home/zp/ros_car/zpzp_openvino_model/zpzp.xml'
DEVICE     = 'GPU'
CONF_THRES = 0.35
NMS_THRES  = 0.45
QUEUE_SZ   = 2

# 动态获取当前脚本所在的绝对路径，确保同目录下的 mp3 能被精准定位
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

AUDIO_MAPPING = {
    'cube': '这个是正方体.mp3',
    'sphere': '这个是球体.mp3',
    'cylinder': '圆柱体.mp3'
}

# 目标类别映射（请根据你 YOLO 模型的实际类别顺序进行对齐）
CLASS_NAMES = {0: 'cube', 1: 'sphere', 2: 'cylinder'}

# ==================== 2. 整合后的机器人控制节点 ====================
class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        
        # 初始化 Nav2 导航控制核心
        self.nav = BasicNavigator()
        
        # 15 个巡航目标点
        self.waypoint_list = [
            [0.577364, 0.0303617, 0.0289215, 0.999582],   # P1
            [1.11061, 0.0419989, -0.63543, 0.772159],     # P2
            [1.23693, -0.55939, -0.179482, 0.983761],     # P3
            [1.62779, -0.580917, 0.390942, 0.920415],     # P4
            [1.7324, 0.0374521, 0.976355, 0.216173],      # P5
            [1.03013, 0.272374, 0.752633, 0.65844],       # P6
            [1.18174, 0.791343, 0.175505, 0.984479],      # P7
            [1.76274, 0.449419, -0.523705, 0.851899],     # P8
            [1.94688, 0.11959, 0.0402934, 0.999188],      # P9  -> 触发识别（静默）
            [2.51021, 0.1545, 0.0374193, 0.9993],          # P10 -> 触发识别并锁定最终顺序（静默）
            [2.58639, 0.1927, 0.0415961, 0.999135],       # P11
            [2.74039, 0.224071, 0.0438426, 0.999038],      # P12
            # ⬇️ 最后的三个目标释放点（在此处才进行对应物体的语音播报）
            [3.13837, -0.430822, 0.714831, 0.699297],     # P13 -> 对应【从右到左】第 1 个物体
            [3.09834, 0.214466, 0.732353, 0.680925],      # P14 -> 对应【从右到左】第 2 个物体
            [3.01064, 0.794791, 0.731985, 0.681321]       # P15 -> 对应【从右到左】第 3 个物体
        ]
        
        # 用于记忆识别到的【从右到左】的物体名称清单
        self.right_to_left_objects = []
        
        # 初始化 OpenVINO 环境
        self.init_openvino()

    def init_openvino(self):
        """初始化 OpenVINO 模型与推理双缓冲队列"""
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
        """YOLOv11 后处理"""
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
        """统一音频播放接口（合并当前脚本所在绝对路径）"""
        audio_path = os.path.join(CURRENT_DIR, AUDIO_MAPPING[name])
        if os.path.exists(audio_path):
            self.get_logger().info(f"🔊 正在播放音频文件: {audio_path}")
            # 使用 play 播放器，并在后台静音无关输出
            os.system(f"play -q {audio_path} &> /dev/null")
        else:
            self.get_logger().error(f"❌ 无法找到目标音频文件: {audio_path}，请检查文件名是否正确。")

    def run_vision_and_save(self, station_idx):
        """在 9, 10 点位运行的静默视觉识别与排序记忆函数（无现场播报）"""
        self.get_logger().info(f"=== [点位 {station_idx}] 启动静默视觉检测推理 ===")
        
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        for _ in range(5): cap.read()  # 预热摄像头

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
            self.get_logger().warn(f"[点位 {station_idx}] 视野内未发现有效物体标靶。")
            return

        # 去噪并求每一类的平均中心 X 坐标
        unique_targets = {}
        for x_center, name in detected_objects:
            if name not in unique_targets:
                unique_targets[name] = []
            unique_targets[name].append(x_center)
            
        # ✨ 核心记忆：计算自右向左排序（X 坐标从大到小），仅存入内存，不进行常规播报
        right_to_left = sorted(
            [(np.mean(xs), name) for name, xs in unique_targets.items()],
            key=lambda item: item[0],
            reverse=True
        )
        self.right_to_left_objects = [item[1] for item in right_to_left]
        self.get_logger().info(f"✨ [静默完成] 成功更新[从右到左]序列内存：{self.right_to_left_objects}。准备在 P13-P15 释放点播报。")

    def run_patrol_flow(self):
        """全自动 15 点巡航控制流"""
        self.get_logger().info("正在等待 Nav2 导航生命周期服务器就绪...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("Nav2 状态正常，开始全自动巡航！")
        
        total_points = len(self.waypoint_list)
        
        for index, coords in enumerate(self.waypoint_list, start=1):
            self.get_logger().info(f"==> 任务目标更新 [{index}/{total_points}]: 正在奔赴坐标点 ({coords[0]}, {coords[1]})")
            
            target_pose = self.create_pose(coords)
            self.nav.goToPose(target_pose)
            
            # 阻塞等待当前点到站
            while not self.nav.isTaskComplete():
                time.sleep(0.1)

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"成功到达第 [{index}] 个点！")
                
                # 情景 A：在 9 和 10 号点位进行静默视觉识别与排序记忆
                if index in [9, 10]:
                    self.run_vision_and_save(station_idx=index)
                    time.sleep(1.0)
                    
                # 情景 B：最后的 13, 14, 15 号专属目标释放点（在这里才会触发语音播报）
                elif index in [13, 14, 15]:
                    obj_idx = index - 13  # 映射索引：13点->倒数第1(索引0), 14点->倒数第2(索引1), 15点->倒数第3(索引2)
                    
                    if obj_idx < len(self.right_to_left_objects):
                        target_name = self.right_to_left_objects[obj_idx]
                        self.get_logger().info(f"🎯 到达对应释放点 P{index} ──> 正在播报[从右向左]第 {obj_idx+1} 个物体语音")
                        self.play_voice(target_name)
                        time.sleep(1.5)  # 播放完毕后原地休整，防止衔接太快
                    else:
                        self.get_logger().error(f"❌ 播报失败：在第 9/10 点位未能成功识别到第 {obj_idx+1} 个物体。")
                
                # 普通点位正常休整
                else:
                    time.sleep(1.0)
                    
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f"警告：第 [{index}] 个点的任务中途被手动取消。")
                break
            elif result == TaskResult.FAILED:
                self.get_logger().error(f"错误：前往第 [{index}] 个点时宣告失败！放弃后续巡航。")
                break

        self.get_logger().info("★ 所有给定的巡航及定向释放播报任务已全部结束。")

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

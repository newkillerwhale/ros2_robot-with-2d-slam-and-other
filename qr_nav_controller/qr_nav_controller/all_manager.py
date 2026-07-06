import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import json
import os
import glob
import time

class AllManager(Node):
    def __init__(self):
        super().__init__('all_manager')
        
        # 路径配置
        self.script_path = "/home/zp/ros_car/ros_car/src/qr_nav_controller/qr_nav_controller"
        self.map_folder = "/home/zp/ros_car/ros_car/src/yahboomcar_nav/maps"
        
        # 订阅/发布
        self.cmd_sub = self.create_subscription(String, '/app/global_cmd', self.global_cmd_callback, 10)
        self.status_pub = self.create_publisher(String, '/app/system_status', 10)
        
        self.active_process = None
        self.current_mode = "Idle"

        self.get_logger().info(">>> 总控节点 all_manager 已启动")
        
        # 初始发送地图储备
        self.timer = self.create_timer(1.5, self.send_map_list)

    def send_map_list(self):
        """扫描地图并推送给 App"""
        maps = []
        if os.path.exists(self.map_folder):
            files = glob.glob(os.path.join(self.map_folder, "*.yaml"))
            maps = [os.path.splitext(os.path.basename(f))[0] for f in files]
        
        msg = String()
        msg.data = json.dumps({"mode": "map_list", "maps": maps})
        self.status_pub.publish(msg)
        self.timer.cancel()

    def global_cmd_callback(self, msg):
        try:
            data = json.loads(msg.data)
            mode = data.get('mode')
            
            if mode == 'mapping':
                self.run_script("start_map.sh", "Mapping", "Switching_To_Mapping")
            elif mode == 'navigation':
                map_name = data.get('map_name', 'yahboomcar')
                self.run_nav_with_map(map_name)
            elif mode == 'task':
                self.run_script("start_task.sh", "Task", "Switching_To_Task")
            elif mode == 'save_map':
                map_name = data.get('map_name', 'yahboomcar')
                self.handle_save_map(map_name)
            elif mode == 'get_maps':
                self.send_map_list()
            elif mode == 'stop':
                self.stop_all()
                
        except Exception as e:
            self.get_logger().error(f"指令解析出错: {e}")

    def stop_all(self):
        """物理级强制清理，解决地图重叠和切换冲突"""
        if self.current_mode == "Idle" and not self.active_process:
            return

        self.get_logger().info(f"正在强制清理当前模式: {self.current_mode}...")
        
        # 告知 App 正在重置，App 应在此状态下清空 UI 地图
        self.publish_feedback("System_Resetting")

        try:
            # 1. 终止主脚本进程
            if self.active_process:
                self.active_process.terminate()
                try:
                    self.active_process.wait(timeout=1.0)
                except:
                    self.active_process.kill()
                self.active_process = None

            # 2. 深度清理黑名单（包含 SLAM、Nav2、Lidar、底层驱动）
            keywords = [
                'ros2', 'nav2', 'yahboomcar', 'sllidar', 'task_manager', 
                'planner_server', 'controller_server', 'bt_navigator', 
                'map_server', 'amcl', 'lifecycle_manager', 'waypoint_follower',
                'behavior_server', 'slam_toolbox', 'sync_slam', 'async_slam'
            ]
            
            pattern = '|'.join(keywords)
            
            # 使用组合命令彻底杀死相关 PID，但保护 rosbridge 和 all_manager
            kill_cmd = (
                f"ps -ef | grep -E '({pattern})' "
                "| grep -v 'grep' "
                "| grep -v 'rosbridge' "
                "| grep -v 'all_manager' "
                "| awk '{{print $2}}' | xargs kill -9 2>/dev/null"
            )
            subprocess.run(kill_cmd, shell=True, check=False)
            
            # 3. 清理残留的 shell 脚本
            subprocess.run("pkill -9 -f start_", shell=True, check=False)

            # --- 关键：物理断流等待 ---
            # 必须等待一段时间，让 ROS2 的网络拓扑彻底注销旧发布者，否则旧地图会顶掉新地图
            time.sleep(1.5)

        except Exception as e:
            self.get_logger().error(f"清理进程异常: {e}")

        self.current_mode = "Idle"
        self.publish_feedback("Idle")
        self.get_logger().info(">>> 环境清理完成，系统已就绪")

    def run_nav_with_map(self, map_name):
        """带参启动导航，确保先死后生"""
        self.stop_all() 
        
        full_path = os.path.join(self.script_path, "start_nav.sh")
        map_file = os.path.join(self.map_folder, f"{map_name}.yaml")
        
        if not os.path.exists(map_file):
            self.publish_feedback(f"Error: Map {map_name} not found")
            return

        try:
            self.current_mode = "Navigation"
            # 传入地图名给 .sh 脚本
            self.active_process = subprocess.Popen(["/bin/bash", full_path, map_name])
            self.publish_feedback(f"Navigation_Starting_{map_name}")
        except Exception as e:
            self.get_logger().error(f"启动导航失败: {e}")

    def run_script(self, script_name, mode_label, feedback_text):
        """通用脚本启动器"""
        self.stop_all()
        full_path = os.path.join(self.script_path, script_name)
        try:
            self.current_mode = mode_label
            self.active_process = subprocess.Popen(["/bin/bash", full_path])
            self.publish_feedback(feedback_text)
        except Exception as e:
            self.get_logger().error(f"启动脚本 {script_name} 失败: {e}")

    def handle_save_map(self, map_name):
        """保存地图"""
        full_save_path = os.path.join(self.map_folder, map_name)
        try:
            subprocess.run(["ros2", "launch", "yahboomcar_nav", "save_map_launch.py", f"map_path:={full_save_path}"], check=True)
            self.publish_feedback(f"Save_Success_{map_name}")
            self.send_map_list()
        except:
            self.publish_feedback("Save_Failed")

    def publish_feedback(self, info):
        msg = String()
        msg.data = json.dumps({"mode": self.current_mode, "info": info})
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AllManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前尝试最后清理一次
        if node.active_process:
            node.active_process.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

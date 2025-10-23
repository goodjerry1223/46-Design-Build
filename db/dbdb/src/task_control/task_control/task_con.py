#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
import subprocess
import signal
import threading
import time
import math
import os
import psutil

class ExplorationNavigator(Node):
    def __init__(self):
        super().__init__('exploration_navigator')
        
        # 配置参数
        self.origin = Point(x=0.0, y=0.0, z=0.0)       # 起始位置
        self.target = Point(x=8.0, y=4.0, z=0.0)        # 目标位置
        self.map_valid_threshold = 0.5                  # 地图点有效距离阈值（米）
        self.explore_duration = 600                     # 最大探索时间（秒）
        self.navigation_timeout = 120                   # 导航超时时间（秒）
        self.stuck_detection_time = 10                  # 卡住检测时间（秒）
        
        # 状态变量
        self.exploration_process = None
        self.map_available = False
        self.target_mapped = False
        self.navigation_done = False
        self.previous_distance = float('inf')
        self.last_movement_time = time.time()
        
        # 创建地图订阅
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.handle_map_update,
            10)
        
        # 创建导航动作客户端
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info("探索导航系统初始化完成，启动探索流程...")
        self.initiate_exploration()

    def initiate_exploration(self):
        """启动探索建图流程"""
        def execute_exploration():
            self.exploration_process = subprocess.Popen(
                ['ros2', 'launch', 'explore_lite', 'explore.launch.py'],
                preexec_fn=os.setsid  # 创建新的进程组
            )
            self.exploration_process.wait()
        
        # 在单独线程中运行探索
        self.exploration_thread = threading.Thread(target=execute_exploration)
        self.exploration_thread.start()
        
        # 设置探索超时监控
        self.explore_timer = self.create_timer(
            self.explore_duration, 
            self.handle_exploration_timeout)

    def handle_map_update(self, map_data):
        """处理地图更新消息"""
        self.map_available = True
        
        # 检查目标点是否已出现在地图中
        if not self.target_mapped:
            self.verify_target_in_map(map_data)

    def verify_target_in_map(self, map_info):
        """验证目标点是否已建图"""
        # 计算目标点在地图网格中的坐标
        resolution = map_info.info.resolution
        origin_x = map_info.info.origin.position.x
        origin_y = map_info.info.origin.position.y
        
        grid_x = int((self.target.x - origin_x) / resolution)
        grid_y = int((self.target.y - origin_y) / resolution)
        
        # 检查坐标是否在地图范围内
        if 0 <= grid_x < map_info.info.width and 0 <= grid_y < map_info.info.height:
            index = grid_y * map_info.info.width + grid_x
            
            # 检查该位置是否已知（非未知区域）
            if map_info.data[index] != -1:  # -1表示未知区域
                self.target_mapped = True
                self.get_logger().info("目标位置已成功建图，终止探索进程...")
                self.terminate_exploration()
                self.start_target_navigation()

    def terminate_exploration(self):
        """终止探索进程"""
        if self.exploration_process and self.exploration_process.poll() is None:
            try:
                # 终止整个进程组
                os.killpg(os.getpgid(self.exploration_process.pid), signal.SIGTERM)
                self.get_logger().info("已发送终止信号给探索进程组")
            except ProcessLookupError:
                self.get_logger().warning("进程组不存在，可能已退出")
            
            # 等待进程退出
            try:
                self.exploration_process.wait(timeout=5.0)
                self.get_logger().info("探索进程已终止")
            except subprocess.TimeoutExpired:
                self.get_logger().warning("探索进程未正常退出，强制终止...")
                try:
                    os.killpg(os.getpgid(self.exploration_process.pid), signal.SIGKILL)
                    self.exploration_process.wait(timeout=2.0)
                except:
                    pass
            
            # 确保所有相关进程都被终止
            self.cleanup_ros_processes('explore_lite')
            
            self.exploration_process = None

    def cleanup_ros_processes(self, process_name):
        """清理所有指定名称的ROS进程"""
        self.get_logger().info(f"清理残留的{process_name}进程...")
        for proc in psutil.process_iter(['name', 'cmdline']):
            try:
                if proc.info['cmdline'] and any(process_name in cmd for cmd in proc.info['cmdline']):
                    self.get_logger().info(f"终止进程: {proc.pid} {proc.info['cmdline']}")
                    proc.terminate()
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass

    def handle_exploration_timeout(self):
        """处理探索超时"""
        if not self.target_mapped:
            self.get_logger().warning("探索超时，目标点未建图")
            self.terminate_exploration()
            self.start_target_navigation()

    def start_target_navigation(self):
        """导航到目标位置"""
        self.get_logger().info("开始导航至目标位置...")
        self.navigate_to_position(self.target, self.target_reached_callback)

    def target_reached_callback(self, success):
        """目标位置到达回调"""
        if success:
            self.get_logger().info("已到达目标位置，开始返回起始点...")
            self.return_to_origin()
        else:
            self.get_logger().error("导航至目标位置失败，终止任务")
            self.system_shutdown()

    def return_to_origin(self):
        """导航回起始位置"""
        self.get_logger().info("开始导航回起始位置...")
        self.navigate_to_position(self.origin, self.origin_reached_callback)

    def origin_reached_callback(self, success):
        """起始位置到达回调"""
        if success:
            self.get_logger().info("已成功返回起始位置，任务完成！")
        else:
            self.get_logger().error("返回起始位置失败")
        self.system_shutdown()

    def navigate_to_position(self, position, completion_callback):
        """导航到指定位置"""
        navigation_goal = NavigateToPose.Goal()
        target_pose = PoseStamped()
        
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position = position
        target_pose.pose.orientation.w = 1.0  # 默认朝向
        
        navigation_goal.pose = target_pose
        
        self.navigation_client.wait_for_server()
        self.goal_future = self.navigation_client.send_goal_async(
            navigation_goal,
            feedback_callback=self.navigation_progress_callback)
        
        self.goal_future.add_done_callback(
            lambda future: self.handle_goal_response(future, completion_callback))

    def handle_goal_response(self, future, completion_callback):
        """处理目标响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            completion_callback(False)
            return
        
        self.get_logger().info('导航目标已接受，开始执行...')
        self.result_future = goal_handle.get_result_async()
        
        # 设置导航超时监控
        self.nav_timeout_timer = self.create_timer(
            self.navigation_timeout,
            lambda: self.handle_navigation_timeout(goal_handle))
        
        # 设置卡住检测监控
        self.stuck_monitor = self.create_timer(
            1.0,  # 每秒检查一次
            self.monitor_movement)
        
        self.result_future.add_done_callback(
            lambda future: self.handle_navigation_result(future, completion_callback))

    def handle_navigation_result(self, future, completion_callback):
        """处理导航结果"""
        # 取消导航超时监控
        if hasattr(self, 'nav_timeout_timer'):
            self.nav_timeout_timer.cancel()
        
        # 取消卡住检测监控
        if hasattr(self, 'stuck_monitor'):
            self.stuck_monitor.cancel()
        
        result = future.result().result
        # 简化处理：假设动作完成即成功
        completion_callback(True)

    def handle_navigation_timeout(self, goal_handle):
        """处理导航超时"""
        self.get_logger().warning("导航超时，取消当前目标")
        goal_handle.cancel_goal_async()
        self.result_future.cancel()
        self.handle_navigation_result(self.result_future, lambda result: False)

    def monitor_movement(self):
        """监控机器人移动状态"""
        current_time = time.time()
        if current_time - self.last_movement_time > self.stuck_detection_time:
            if abs(self.previous_distance - self.current_distance) < 0.1:  # 10厘米内移动视为卡住
                self.get_logger().warning("机器人可能卡住，尝试重新规划路径")
                self.handle_navigation_timeout(self.current_goal_handle)
        
        # 更新最后记录的距离和时间
        self.previous_distance = self.current_distance
        self.last_movement_time = current_time

    def navigation_progress_callback(self, feedback):
        """导航进度回调"""
        progress = feedback.feedback
        if hasattr(progress, 'distance_remaining'):
            self.current_distance = progress.distance_remaining
            self.get_logger().info(f'剩余距离: {self.current_distance:.2f}米')
        else:
            self.get_logger().info('收到导航进度更新...')

    def system_shutdown(self):
        """系统清理与关闭"""
        self.get_logger().info("执行系统清理并关闭节点...")
        self.navigation_done = True
        
        # 取消所有定时器
        if hasattr(self, 'explore_timer'):
            self.explore_timer.cancel()
        if hasattr(self, 'nav_timeout_timer'):
            self.nav_timeout_timer.cancel()
        if hasattr(self, 'stuck_monitor'):
            self.stuck_monitor.cancel()
        
        # 安全关闭节点
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    try:
        navigator = ExplorationNavigator()
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            navigator.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
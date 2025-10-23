#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import re
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

class LidarSerialNode(Node):
    def __init__(self):
        super().__init__('lidar_serial_publisher')

        # === 参数配置 ===
        self.declare_parameter('serial_port', '/dev/rfcomm0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('scan_frame', 'laser_frame')
        self.declare_parameter('publish_frequency', 10.0)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.scan_frame = self.get_parameter('scan_frame').get_parameter_value().string_value
        self.pub_rate = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # === 初始化串口连接 ===
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'串口连接成功: {port}@{baud}bps')
        except Exception as e:
            self.get_logger().error(f'串口连接失败: {e}')
            exit(1)

        # === 雷达数据处理 ===
        self.scan_values = [float('inf')] * 360
        self.data_pattern = re.compile(r'Angle:\s*([\d\.]+)\s*deg\s*Dist:\s*([\d\.]+)\s*mm')

        # === 创建定时器发布雷达数据 ===
        self.scan_timer = self.create_timer(1.0 / self.pub_rate, self.process_and_publish)
        self.last_status_time = time.time()

        # === 创建速度指令订阅 ===
        self.velocity_sub = self.create_subscription(Twist, 'cmd_vel', self.handle_velocity_command, 10)
        self.last_direction = 5  # 初始为停止（松开）
        
        # === 运动控制相关 ===
        self.release_timer = None  # 松开指令定时器
        self.current_command = 5   # 当前指令（默认松开）

    # ============ 雷达数据处理与发布 =============
    def process_and_publish(self):
        try:
            # 读取并处理串口数据
            raw_data = self.serial_conn.readlines()
            self.parse_serial_data(raw_data)
            
            # 创建并发布LaserScan消息
            scan_msg = self.create_scan_message()
            self.scan_publisher.publish(scan_msg)
            
            # 限频状态日志
            current_time = time.time()
            if current_time - self.last_status_time > 2.0:
                self.get_logger().info('雷达数据已发布')
                self.last_status_time = current_time

        except Exception as e:
            self.get_logger().error(f'数据处理异常: {e}')

    def parse_serial_data(self, raw_lines):
        """解析串口数据并更新扫描值"""
        for line in raw_lines:
            try:
                decoded_line = line.decode('utf-8', errors='ignore')
                match = self.data_pattern.search(decoded_line)
                if match:
                    angle = float(match.group(1))
                    distance = float(match.group(2))
                    index = int(angle) % 360
                    distance_m = distance / 1000.0
                    
                    # 过滤无效数据
                    if distance_m <= 0.05 or distance_m > 20.0:
                        distance_m = float('inf')
                    
                    self.scan_values[index] = distance_m
            except Exception as e:
                self.get_logger().warn(f'数据解析异常: {e}')

    def create_scan_message(self):
        """create LaserScan message from scan values"""
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.scan_frame
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2 * math.pi
        scan_msg.angle_increment = math.radians(1.0)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self.pub_rate
        scan_msg.range_min = 0.05
        scan_msg.range_max = 20.0
        scan_msg.ranges = self.scan_values
        return scan_msg

    # ============ 速度指令处理 =============
    def handle_velocity_command(self, velocity_msg):
        """处理速度指令"""
        linear_velocity = velocity_msg.linear.x
        angular_velocity = velocity_msg.angular.z
        direction = self.determine_direction(linear_velocity, angular_velocity)
        
        # 仅当方向变化时发送指令
        if direction != self.last_direction:
            self.send_movement_command(direction)
            self.last_direction = direction

    def determine_direction(self, linear, angular):
        """根据速度确定移动方向，返回按键编码"""
        if abs(linear) < 0.01 and abs(angular) < 0.01:
            return 5  # 停止（松开）
        elif linear > 0.01:
            return 2  # 前进（按下）
        elif linear < -0.01:
            return 8  # 后退（按下）
        elif angular > 0.01:
            return 4  # 左转（按下）
        elif angular < -0.01:
            return 6  # 右转（按下）
        return self.last_direction  # 默认保持上次方向

    def send_movement_command(self, command):
        """通过串口发送移动指令（按键编码）"""
        try:
            # 发送按键编码
            command_str = str(command)
            self.serial_conn.write(command_str.encode('utf-8'))
            self.current_command = command
            
            # 记录指令类型
            command_names = {2: '前进', 4: '左转', 5: '松开', 6: '右转', 8: '后退'}
            command_name = command_names.get(command, f'未知({command})')
            self.get_logger().info(f'发送移动指令: {command} ({command_name})')
            
            # 如果是运动指令（非松开），设置定时器自动松开
            if command != 5:
                self.schedule_release_command()
                
        except Exception as e:
            self.get_logger().error(f'指令发送失败: {e}')
    
    def schedule_release_command(self):
        """安排松开指令的发送"""
        # 取消之前的定时器
        if self.release_timer is not None:
            self.release_timer.cancel()
        
        # 创建新的定时器，0.1秒后发送松开指令
        self.release_timer = self.create_timer(0.1, self.send_release_command)
    
    def send_release_command(self):
        """发送松开指令"""
        if self.current_command != 5:  # 只有在当前不是松开状态时才发送
            try:
                self.serial_conn.write('5'.encode('utf-8'))
                self.current_command = 5
                self.get_logger().info('发送松开指令: 5')
            except Exception as e:
                self.get_logger().error(f'松开指令发送失败: {e}')
        
        # 取消定时器
        if self.release_timer is not None:
            self.release_timer.cancel()
            self.release_timer = None

def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarSerialNode()
    
    try:
        rclpy.spin(lidar_node)
    except KeyboardInterrupt:
        lidar_node.get_logger().info('节点终止')
    finally:
        lidar_node.serial_conn.close()
        lidar_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
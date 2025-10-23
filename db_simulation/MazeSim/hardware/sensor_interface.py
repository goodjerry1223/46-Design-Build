from abc import ABC, abstractmethod
from typing import Tuple, List, Set
import numpy as np
import time

class SensorData:
    """传感器数据标准格式"""
    
    def __init__(self, angles: List[float], distances: List[float], timestamp: float = None):
        self.angles = angles
        self.distances = distances
        self.timestamp = timestamp or time.time()
        self.scan_size = len(angles)
    
    def to_dict(self) -> dict:
        """转换为字典格式"""
        return {
            'angles': self.angles,
            'distances': self.distances,
            'timestamp': self.timestamp,
            'scan_size': self.scan_size
        }
    
    @classmethod
    def from_dict(cls, data: dict):
        """从字典创建对象"""
        return cls(
            angles=data['angles'],
            distances=data['distances'],
            timestamp=data.get('timestamp')
        )

class LidarInterface(ABC):
    """雷达传感器接口抽象类"""
    
    @abstractmethod
    def get_scan_data(self) -> SensorData:
        """获取扫描数据"""
        pass
    
    @abstractmethod
    def is_connected(self) -> bool:
        """检查连接状态"""
        pass
    
    @abstractmethod
    def initialize(self) -> bool:
        """初始化传感器"""
        pass
    
    @abstractmethod
    def shutdown(self):
        """关闭传感器"""
        pass

class SimulatedLidar(LidarInterface):
    """模拟雷达传感器"""
    
    def __init__(self, maze=None, position=None, scan_size=32, max_dist=6):
        self.maze = maze
        self.position = position
        self.scan_size = scan_size
        self.max_dist = max_dist
        self.connected = False
    
    def set_environment(self, maze, position):
        """设置环境和位置"""
        self.maze = maze
        self.position = position
    
    def get_scan_data(self) -> SensorData:
        """获取模拟扫描数据"""
        if not self.connected or self.maze is None or self.position is None:
            return SensorData([], [])
        
        visible, angles, distances = self._simulate_lidar_with_distances(
            self.position, self.maze, self.scan_size, self.max_dist
        )
        
        return SensorData(angles.tolist(), distances)
    
    def _simulate_lidar_with_distances(self, pos, maze, scan_size=32, max_dist=6):
        """模拟雷达扫描（从原有代码移植）"""
        visible = set()
        x, y = pos
        h, w = maze.shape
        angles = np.linspace(0, 2*np.pi, scan_size, endpoint=False)
        distances = []
        
        for theta in angles:
            hit_distance = max_dist
            for d in range(1, max_dist+1):
                nx = int(round(x + d*np.cos(theta)))
                ny = int(round(y + d*np.sin(theta)))
                if 0 <= nx < h and 0 <= ny < w:
                    visible.add((nx, ny))
                    if maze[nx, ny] == 1:
                        hit_distance = d
                        break
                else:
                    hit_distance = d - 1 if d > 1 else 0
                    break
            distances.append(hit_distance)
        
        return visible, angles, distances
    
    def is_connected(self) -> bool:
        return self.connected
    
    def initialize(self) -> bool:
        self.connected = True
        return True
    
    def shutdown(self):
        self.connected = False

class BluetoothLidar(LidarInterface):
    """蓝牙雷达传感器"""
    
    def __init__(self, bluetooth_manager):
        self.bluetooth_manager = bluetooth_manager
        self.last_scan_data = None
    
    def get_scan_data(self) -> SensorData:
        """获取蓝牙雷达数据"""
        if not self.is_connected():
            return SensorData([], [])
        
        # 从蓝牙管理器获取传感器数据
        sensor_data = self.bluetooth_manager.get_sensor_data()
        
        if sensor_data and sensor_data.get('type') == 'sensor_data':
            lidar_data = sensor_data.get('lidar', {})
            angles = lidar_data.get('angles', [])
            distances = lidar_data.get('distances', [])
            
            if angles and distances:
                return SensorData(angles, distances, sensor_data.get('timestamp'))
        
        # 如果没有新数据，返回空数据
        return SensorData([], [])
    
    def is_connected(self) -> bool:
        return self.bluetooth_manager.is_connected()
    
    def initialize(self) -> bool:
        return self.bluetooth_manager.is_connected()
    
    def shutdown(self):
        pass

class MotionController:
    """运动控制器接口"""
    
    def __init__(self, bluetooth_manager=None, simulation_mode=True):
        self.bluetooth_manager = bluetooth_manager
        self.simulation_mode = simulation_mode
    
    def send_velocity_command(self, linear_vel: float, angular_vel: float) -> bool:
        """发送速度控制指令"""
        if self.simulation_mode:
            # 仿真模式下只打印指令
            print(f"仿真控制指令 - 线速度: {linear_vel:.3f}, 角速度: {angular_vel:.3f}")
            return True
        else:
            # 硬件模式下通过蓝牙发送
            if self.bluetooth_manager and self.bluetooth_manager.is_connected():
                return self.bluetooth_manager.send_control(linear_vel, angular_vel)
            return False
    
    def emergency_stop(self) -> bool:
        """紧急停止"""
        return self.send_velocity_command(0.0, 0.0)
    
    def is_ready(self) -> bool:
        """检查控制器是否就绪"""
        if self.simulation_mode:
            return True
        return self.bluetooth_manager and self.bluetooth_manager.is_connected()
import time
from typing import Optional
from .bluetooth_driver import BluetoothManager
from .sensor_interface import LidarInterface, SimulatedLidar, BluetoothLidar, MotionController
import config

class HardwareManager:
    """硬件管理器 - 统一管理所有硬件组件"""
    
    def __init__(self):
        self.bluetooth_manager: Optional[BluetoothManager] = None
        self.lidar: Optional[LidarInterface] = None
        self.motion_controller: Optional[MotionController] = None
        self.hardware_mode = config.HARDWARE_MODE
        self.initialized = False
        
    def initialize(self, maze=None, position=None) -> bool:
        """初始化硬件管理器"""
        print(f"初始化硬件管理器 - 模式: {'硬件' if self.hardware_mode else '仿真'}")
        
        if self.hardware_mode:
            # 硬件模式：初始化蓝牙连接
            success = self._initialize_hardware_mode()
        else:
            # 仿真模式：初始化模拟器
            success = self._initialize_simulation_mode(maze, position)
        
        if success:
            self.initialized = True
            print("硬件管理器初始化成功")
        else:
            print("硬件管理器初始化失败")
        
        return success
    
    def _initialize_hardware_mode(self) -> bool:
        """初始化硬件模式"""
        try:
            # 初始化蓝牙管理器
            self.bluetooth_manager = BluetoothManager()
            
            # 获取蓝牙配置
            bt_config = config.BLUETOOTH_CONFIG
            
            # 尝试连接蓝牙设备
            success = self.bluetooth_manager.initialize(
                connection_type=bt_config['connection_type'],
                com_port=bt_config['com_port'],
                baudrate=bt_config['baudrate'],
                device_address=bt_config['device_address']
            )
            
            if not success:
                print("蓝牙连接失败")
                return False
            
            # 初始化蓝牙雷达
            self.lidar = BluetoothLidar(self.bluetooth_manager)
            
            # 初始化运动控制器
            self.motion_controller = MotionController(
                bluetooth_manager=self.bluetooth_manager,
                simulation_mode=False
            )
            
            return True
            
        except Exception as e:
            print(f"硬件模式初始化失败: {e}")
            return False
    
    def _initialize_simulation_mode(self, maze=None, position=None) -> bool:
        """初始化仿真模式"""
        try:
            # 初始化模拟雷达
            sensor_config = config.SENSOR_CONFIG
            self.lidar = SimulatedLidar(
                maze=maze,
                position=position,
                scan_size=sensor_config['lidar_scan_size'],
                max_dist=sensor_config['lidar_max_distance']
            )
            
            # 初始化运动控制器（仿真模式）
            self.motion_controller = MotionController(
                bluetooth_manager=None,
                simulation_mode=True
            )
            
            # 初始化传感器
            return self.lidar.initialize()
            
        except Exception as e:
            print(f"仿真模式初始化失败: {e}")
            return False
    
    def update_simulation_environment(self, maze, position):
        """更新仿真环境（仅在仿真模式下有效）"""
        if not self.hardware_mode and isinstance(self.lidar, SimulatedLidar):
            self.lidar.set_environment(maze, position)
    
    def get_lidar_data(self):
        """获取雷达数据"""
        if not self.initialized or not self.lidar:
            return None, [], []
        
        try:
            sensor_data = self.lidar.get_scan_data()
            
            if sensor_data.angles and sensor_data.distances:
                # 如果是仿真模式，还需要返回可见格子信息
                if not self.hardware_mode and isinstance(self.lidar, SimulatedLidar):
                    # 重新计算可见格子（为了兼容现有代码）
                    visible, _, _ = self.lidar._simulate_lidar_with_distances(
                        self.lidar.position, self.lidar.maze,
                        self.lidar.scan_size, self.lidar.max_dist
                    )
                    return visible, sensor_data.angles, sensor_data.distances
                else:
                    # 硬件模式下没有可见格子概念
                    return set(), sensor_data.angles, sensor_data.distances
            
        except Exception as e:
            print(f"获取雷达数据失败: {e}")
        
        return set(), [], []
    
    def send_motion_command(self, linear_vel: float, angular_vel: float) -> bool:
        """发送运动控制指令"""
        if not self.initialized or not self.motion_controller:
            return False
        
        # 限制速度范围
        motion_config = config.MOTION_CONFIG
        linear_vel = max(-motion_config['max_linear_vel'], 
                        min(motion_config['max_linear_vel'], linear_vel))
        angular_vel = max(-motion_config['max_angular_vel'], 
                         min(motion_config['max_angular_vel'], angular_vel))
        
        return self.motion_controller.send_velocity_command(linear_vel, angular_vel)
    
    def emergency_stop(self) -> bool:
        """紧急停止"""
        if self.motion_controller:
            return self.motion_controller.emergency_stop()
        return False
    
    def is_connected(self) -> bool:
        """检查连接状态"""
        if not self.initialized:
            return False
        
        if self.hardware_mode:
            return (self.bluetooth_manager and self.bluetooth_manager.is_connected() and
                   self.lidar and self.lidar.is_connected())
        else:
            return self.lidar and self.lidar.is_connected()
    
    def get_status(self) -> dict:
        """获取硬件状态信息"""
        return {
            'initialized': self.initialized,
            'hardware_mode': self.hardware_mode,
            'connected': self.is_connected(),
            'bluetooth_connected': self.bluetooth_manager.is_connected() if self.bluetooth_manager else False,
            'lidar_connected': self.lidar.is_connected() if self.lidar else False,
            'motion_ready': self.motion_controller.is_ready() if self.motion_controller else False
        }
    
    def shutdown(self):
        """关闭硬件管理器"""
        print("关闭硬件管理器...")
        
        if self.motion_controller:
            self.motion_controller.emergency_stop()
        
        if self.lidar:
            self.lidar.shutdown()
        
        if self.bluetooth_manager:
            self.bluetooth_manager.disconnect()
        
        self.initialized = False
        print("硬件管理器已关闭")
    
    def switch_mode(self, hardware_mode: bool, maze=None, position=None) -> bool:
        """切换硬件/仿真模式"""
        if self.hardware_mode == hardware_mode:
            return True
        
        print(f"切换模式: {'仿真' if self.hardware_mode else '硬件'} -> {'硬件' if hardware_mode else '仿真'}")
        
        # 关闭当前模式
        self.shutdown()
        
        # 切换模式
        self.hardware_mode = hardware_mode
        
        # 重新初始化
        return self.initialize(maze, position)
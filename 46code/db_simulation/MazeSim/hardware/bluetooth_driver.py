import serial
import threading
import time
import json
from typing import Optional, Callable, Tuple, List

# 可选的蓝牙模块导入
try:
    import bluetooth
    BLUETOOTH_AVAILABLE = True
except ImportError:
    BLUETOOTH_AVAILABLE = False
    print("警告: bluetooth模块未安装，蓝牙套接字功能将不可用")
    print("如需使用蓝牙套接字，请运行: pip install pybluez")

class BluetoothDriver:
    """蓝牙通信驱动类"""
    
    def __init__(self, device_address: str = None, port: int = 1):
        self.device_address = device_address
        self.port = port
        self.socket = None
        self.serial_conn = None
        self.is_connected = False
        self.receive_thread = None
        self.running = False
        self.data_callback = None
        
    def connect_bluetooth_socket(self, device_address: str) -> bool:
        """通过蓝牙套接字连接设备"""
        if not BLUETOOTH_AVAILABLE:
            print("蓝牙模块不可用，无法建立套接字连接")
            return False
            
        try:
            self.socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.socket.connect((device_address, self.port))
            self.is_connected = True
            print(f"蓝牙套接字连接成功: {device_address}")
            return True
        except Exception as e:
            print(f"蓝牙套接字连接失败: {e}")
            return False
    
    def connect_serial_bluetooth(self, com_port: str, baudrate: int = 9600) -> bool:
        """通过串口连接蓝牙设备"""
        try:
            self.serial_conn = serial.Serial(com_port, baudrate, timeout=1)
            self.is_connected = True
            print(f"串口蓝牙连接成功: {com_port}")
            return True
        except Exception as e:
            print(f"串口蓝牙连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=2)
        
        if self.socket:
            self.socket.close()
            self.socket = None
        
        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None
        
        self.is_connected = False
        print("蓝牙连接已断开")
    
    def send_data(self, data: dict) -> bool:
        """发送数据"""
        if not self.is_connected:
            return False
        
        try:
            json_data = json.dumps(data) + '\n'
            
            if self.socket:
                self.socket.send(json_data.encode('utf-8'))
            elif self.serial_conn:
                self.serial_conn.write(json_data.encode('utf-8'))
            
            return True
        except Exception as e:
            print(f"数据发送失败: {e}")
            return False
    
    def start_receiving(self, callback: Callable[[dict], None]):
        """开始接收数据"""
        self.data_callback = callback
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
    
    def _receive_loop(self):
        """数据接收循环"""
        buffer = ""
        
        while self.running and self.is_connected:
            try:
                if self.socket:
                    data = self.socket.recv(1024).decode('utf-8')
                elif self.serial_conn:
                    data = self.serial_conn.read(1024).decode('utf-8')
                else:
                    continue
                
                if data:
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            try:
                                json_data = json.loads(line.strip())
                                if self.data_callback:
                                    self.data_callback(json_data)
                            except json.JSONDecodeError:
                                print(f"无效的JSON数据: {line}")
                
            except Exception as e:
                print(f"数据接收错误: {e}")
                time.sleep(0.1)
    
    def send_control_command(self, linear_vel: float, angular_vel: float) -> bool:
        """发送控制指令"""
        command = {
            'type': 'control',
            'linear_vel': linear_vel,
            'angular_vel': angular_vel,
            'timestamp': time.time()
        }
        return self.send_data(command)
    
    def request_sensor_data(self) -> bool:
        """请求传感器数据"""
        request = {
            'type': 'sensor_request',
            'timestamp': time.time()
        }
        return self.send_data(request)
    
    @staticmethod
    def discover_devices() -> List[Tuple[str, str]]:
        """发现附近的蓝牙设备"""
        try:
            devices = bluetooth.discover_devices(lookup_names=True)
            return devices
        except Exception as e:
            print(f"设备发现失败: {e}")
            return []

class BluetoothManager:
    """蓝牙管理器"""
    
    def __init__(self):
        self.driver = None
        self.last_sensor_data = None
        self.connection_status = False
        
    def initialize(self, connection_type: str = 'serial', **kwargs) -> bool:
        """初始化蓝牙连接"""
        self.driver = BluetoothDriver()
        
        if connection_type == 'serial':
            com_port = kwargs.get('com_port', 'COM3')
            baudrate = kwargs.get('baudrate', 9600)
            success = self.driver.connect_serial_bluetooth(com_port, baudrate)
        elif connection_type == 'socket':
            device_address = kwargs.get('device_address')
            if not device_address:
                print("需要提供设备地址")
                return False
            success = self.driver.connect_bluetooth_socket(device_address)
        else:
            print("不支持的连接类型")
            return False
        
        if success:
            self.driver.start_receiving(self._handle_received_data)
            self.connection_status = True
        
        return success
    
    def _handle_received_data(self, data: dict):
        """处理接收到的数据"""
        if data.get('type') == 'sensor_data':
            self.last_sensor_data = data
    
    def get_sensor_data(self) -> Optional[dict]:
        """获取最新的传感器数据"""
        if self.driver and self.connection_status:
            self.driver.request_sensor_data()
            # 等待数据返回
            timeout = 0.1
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.last_sensor_data:
                    data = self.last_sensor_data
                    self.last_sensor_data = None
                    return data
                time.sleep(0.01)
        return None
    
    def send_control(self, linear_vel: float, angular_vel: float) -> bool:
        """发送控制指令"""
        if self.driver and self.connection_status:
            return self.driver.send_control_command(linear_vel, angular_vel)
        return False
    
    def is_connected(self) -> bool:
        """检查连接状态"""
        return self.connection_status and self.driver and self.driver.is_connected
    
    def disconnect(self):
        """断开连接"""
        if self.driver:
            self.driver.disconnect()
        self.connection_status = False
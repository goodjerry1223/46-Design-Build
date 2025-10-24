# 地图尺寸（单位：格子）
MAP_WIDTH = 9
MAP_HEIGHT = 9

# 模拟雷达参数
LIDAR_ANGLE_RESOLUTION = 1  # 每度一束，共360束
LIDAR_MAX_DISTANCE = 3000   # 最大距离，单位 mm

# SLAM 参数
MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 10.0

# 导航目标点（可替换为动态）
GOAL_X = 8
GOAL_Y = 8

# 硬件模式配置
HARDWARE_MODE = False  # True为硬件模式，False为仿真模式

# 蓝牙配置
BLUETOOTH_CONFIG = {
    'connection_type': 'serial',  # 'serial' 或 'socket'
    'com_port': 'COM3',          # 串口号（Windows）
    'baudrate': 9600,            # 波特率
    'device_address': None,      # 蓝牙设备地址（socket模式使用）
    'auto_reconnect': True,      # 自动重连
    'timeout': 5.0              # 连接超时时间
}

# 传感器配置
SENSOR_CONFIG = {
    'lidar_scan_size': 32,       # 雷达扫描点数
    'lidar_max_distance': 6,     # 雷达最大距离
    'update_rate': 10,           # 传感器更新频率 Hz
    'noise_enabled': True,       # 是否添加噪声
    'noise_std_dev': 0.1        # 噪声标准差
}

# 运动控制配置
MOTION_CONFIG = {
    'max_linear_vel': 0.5,       # 最大线速度 m/s
    'max_angular_vel': 1.0,      # 最大角速度 rad/s
    'control_frequency': 20,     # 控制频率 Hz
    'emergency_stop_enabled': True  # 紧急停止功能
}
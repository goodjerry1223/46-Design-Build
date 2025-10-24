# MazeSim 蓝牙通信模块

本模块为 MazeSim 项目添加了蓝牙通信功能，支持与真实硬件机器人进行无线通信。

## 功能特性

- **双向蓝牙通信**：支持蓝牙串口和套接字连接
- **实时传感器数据**：获取雷达扫描数据和其他传感器信息
- **运动控制指令**：发送线速度和角速度控制命令
- **紧急停止功能**：安全的紧急制动机制
- **模式切换**：支持仿真模式和硬件模式无缝切换

## 文件结构

```
hardware/
├── __init__.py              # 模块初始化
├── bluetooth_driver.py      # 蓝牙驱动核心
├── sensor_interface.py      # 传感器接口抽象
├── hardware_manager.py      # 硬件管理器
├── arduino_example.ino      # Arduino示例代码
└── README.md               # 本文档
```

## 配置说明

在 `config.py` 中可以配置以下参数：

### 硬件模式开关
```python
HARDWARE_MODE = False  # True: 硬件模式, False: 仿真模式
```

### 蓝牙连接配置
```python
BLUETOOTH_CONFIG = {
    'connection_type': 'serial',  # 'serial' 或 'socket'
    'port': 'COM3',              # 串口号 (Windows)
    'address': '00:00:00:00:00:00',  # 蓝牙MAC地址
    'baudrate': 115200,          # 波特率
    'timeout': 5.0,              # 连接超时
    'retry_attempts': 3          # 重试次数
}
```

### 传感器配置
```python
SENSOR_CONFIG = {
    'lidar_scan_points': 32,     # 雷达扫描点数
    'lidar_max_distance': 6.0,   # 最大扫描距离
    'update_frequency': 10,      # 更新频率 (Hz)
    'noise_level': 0.1           # 噪声水平
}
```

## 使用方法

### 1. 仿真模式（默认）

```python
# 在 config.py 中设置
HARDWARE_MODE = False

# 运行程序
python main1.py
```

### 2. 硬件模式

```python
# 在 config.py 中设置
HARDWARE_MODE = True

# 配置蓝牙连接参数
BLUETOOTH_CONFIG = {
    'connection_type': 'serial',
    'port': 'COM3',  # 根据实际情况修改
    'baudrate': 115200,
    'timeout': 5.0,
    'retry_attempts': 3
}

# 运行程序
python main1.py
```

## 硬件端实现

### Arduino/ESP32 示例

参考 `arduino_example.ino` 文件，实现以下功能：

1. **蓝牙服务器设置**
   ```cpp
   BluetoothSerial SerialBT;
   SerialBT.begin("MazeSim_Robot");
   ```

2. **JSON通信协议**
   - 接收运动控制指令
   - 发送传感器数据
   - 处理紧急停止命令

3. **传感器数据采集**
   - 雷达扫描数据
   - 电池电压
   - 温度等其他传感器

4. **电机控制**
   - PWM速度控制
   - 差分驱动实现

## 通信协议

### 运动控制指令（PC → 硬件）
```json
{
    "command": "motion_control",
    "linear_vel": 1.0,
    "angular_vel": 0.5,
    "timestamp": 1234567890
}
```

### 雷达数据请求（PC → 硬件）
```json
{
    "command": "get_lidar_data",
    "timestamp": 1234567890
}
```

### 传感器数据响应（硬件 → PC）
```json
{
    "type": "sensor_data",
    "timestamp": 1234567890,
    "lidar_angles": [0, 11.25, 22.5, ...],
    "lidar_distances": [2.5, 3.1, 1.8, ...],
    "battery_voltage": 12.3,
    "temperature": 25.6
}
```

### 紧急停止（PC → 硬件）
```json
{
    "command": "emergency_stop",
    "timestamp": 1234567890
}
```

## 故障排除

### 常见问题

1. **蓝牙连接失败**
   - 检查蓝牙设备是否已配对
   - 确认串口号或MAC地址正确
   - 检查硬件端是否正常运行

2. **数据传输异常**
   - 检查JSON格式是否正确
   - 确认波特率设置一致
   - 检查超时设置是否合理

3. **运动控制无响应**
   - 检查硬件端电机驱动
   - 确认控制指令格式正确
   - 检查紧急停止状态

### 调试模式

在代码中启用详细日志输出：
```python
# 在 hardware_manager.py 中
logging.basicConfig(level=logging.DEBUG)
```

## 扩展功能

### 添加新传感器

1. 在 `sensor_interface.py` 中扩展 `SensorData` 类
2. 在硬件端添加相应的数据采集代码
3. 更新通信协议

### 支持其他通信方式

1. 在 `bluetooth_driver.py` 中添加新的连接类型
2. 实现相应的驱动接口
3. 更新配置选项

## 安全注意事项

- 始终测试紧急停止功能
- 在硬件模式下注意机器人运动安全
- 定期检查电池电压和温度
- 确保通信稳定性

## 依赖库

```bash
# Python端
pip install pyserial
pip install bluetooth-python  # 可选，用于蓝牙套接字

# Arduino端
# ArduinoJson库
# BluetoothSerial库（ESP32）
```
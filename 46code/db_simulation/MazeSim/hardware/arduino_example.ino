/*
 * MazeSim 硬件端示例代码 (Arduino/ESP32)
 * 实现蓝牙通信和传感器数据采集
 */

#include "BluetoothSerial.h"
#include <ArduinoJson.h>

BluetoothSerial SerialBT;

// 模拟雷达数据结构
struct LidarData {
    float angles[360];
    float distances[360];
    int point_count;
};

// 运动控制变量
float current_linear_vel = 0.0;
float current_angular_vel = 0.0;

void setup() {
    Serial.begin(115200);
    SerialBT.begin("MazeSim_Robot"); // 蓝牙设备名称
    Serial.println("MazeSim 机器人已启动，等待蓝牙连接...");
    
    // 初始化传感器和电机
    initializeSensors();
    initializeMotors();
}

void loop() {
    // 处理蓝牙通信
    handleBluetoothCommunication();
    
    // 执行运动控制
    executeMotionControl();
    
    // 定期发送传感器数据
    static unsigned long lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate > 100) { // 10Hz更新频率
        sendSensorData();
        lastSensorUpdate = millis();
    }
    
    delay(10);
}

void handleBluetoothCommunication() {
    if (SerialBT.available()) {
        String receivedData = SerialBT.readString();
        receivedData.trim();
        
        // 解析JSON命令
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, receivedData);
        
        if (!error) {
            String command = doc["command"];
            
            if (command == "motion_control") {
                current_linear_vel = doc["linear_vel"];
                current_angular_vel = doc["angular_vel"];
                Serial.printf("收到运动指令: 线速度=%.2f, 角速度=%.2f\n", 
                             current_linear_vel, current_angular_vel);
            }
            else if (command == "get_lidar_data") {
                sendLidarData();
            }
            else if (command == "emergency_stop") {
                emergencyStop();
            }
        }
    }
}

void sendSensorData() {
    DynamicJsonDocument doc(2048);
    doc["type"] = "sensor_data";
    doc["timestamp"] = millis();
    
    // 添加雷达数据
    JsonArray angles = doc.createNestedArray("lidar_angles");
    JsonArray distances = doc.createNestedArray("lidar_distances");
    
    LidarData lidar = getLidarData();
    for (int i = 0; i < lidar.point_count; i++) {
        angles.add(lidar.angles[i]);
        distances.add(lidar.distances[i]);
    }
    
    // 添加其他传感器数据
    doc["battery_voltage"] = getBatteryVoltage();
    doc["temperature"] = getTemperature();
    
    String jsonString;
    serializeJson(doc, jsonString);
    SerialBT.println(jsonString);
}

void sendLidarData() {
    DynamicJsonDocument doc(2048);
    doc["type"] = "lidar_response";
    doc["timestamp"] = millis();
    
    JsonArray angles = doc.createNestedArray("angles");
    JsonArray distances = doc.createNestedArray("distances");
    
    LidarData lidar = getLidarData();
    for (int i = 0; i < lidar.point_count; i++) {
        angles.add(lidar.angles[i]);
        distances.add(lidar.distances[i]);
    }
    
    String jsonString;
    serializeJson(doc, jsonString);
    SerialBT.println(jsonString);
}

void initializeSensors() {
    // 初始化雷达传感器
    Serial.println("初始化传感器...");
    // 这里添加具体的传感器初始化代码
}

void initializeMotors() {
    // 初始化电机驱动
    Serial.println("初始化电机...");
    // 这里添加具体的电机初始化代码
}

LidarData getLidarData() {
    LidarData data;
    data.point_count = 32; // 简化为32个点
    
    // 模拟雷达数据生成
    for (int i = 0; i < data.point_count; i++) {
        data.angles[i] = (float)i * (360.0 / data.point_count);
        data.distances[i] = random(50, 400) / 100.0; // 0.5-4.0米
    }
    
    return data;
}

void executeMotionControl() {
    // 根据接收到的速度指令控制电机
    // 这里添加具体的电机控制代码
    
    // 示例：PWM控制
    int leftMotorSpeed = (int)(current_linear_vel * 255 - current_angular_vel * 100);
    int rightMotorSpeed = (int)(current_linear_vel * 255 + current_angular_vel * 100);
    
    // 限制PWM范围
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
    
    // 这里添加实际的电机控制代码
    // analogWrite(LEFT_MOTOR_PIN, abs(leftMotorSpeed));
    // analogWrite(RIGHT_MOTOR_PIN, abs(rightMotorSpeed));
}

void emergencyStop() {
    current_linear_vel = 0.0;
    current_angular_vel = 0.0;
    
    // 立即停止所有电机
    // analogWrite(LEFT_MOTOR_PIN, 0);
    // analogWrite(RIGHT_MOTOR_PIN, 0);
    
    Serial.println("紧急停止执行");
    
    // 发送确认消息
    DynamicJsonDocument doc(256);
    doc["type"] = "emergency_stop_ack";
    doc["timestamp"] = millis();
    
    String jsonString;
    serializeJson(doc, jsonString);
    SerialBT.println(jsonString);
}

float getBatteryVoltage() {
    // 读取电池电压
    return 12.0 + random(-100, 100) / 100.0; // 模拟11.0-13.0V
}

float getTemperature() {
    // 读取温度
    return 25.0 + random(-50, 50) / 10.0; // 模拟20-30°C
}
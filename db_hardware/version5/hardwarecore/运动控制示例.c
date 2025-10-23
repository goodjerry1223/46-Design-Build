/**
 * @file 运动控制示例.c
 * @brief 小车运动控制完整示例
 */

#include "motor_control.h"

/**
 * @brief 小车运动控制示例
 */
void car_motion_example(void)
{
    // 1. 直线前进
    motorStraight(800);     // 80%速度前进
    HAL_Delay(2000);        // 前进2秒
    motorStraight(0);       // 停止
    
    HAL_Delay(1000);        // 等待1秒
    
    // 2. 左转90度
    motorTurnLeft(1000);    // 左转1秒
    
    HAL_Delay(1000);        // 等待1秒
    
    // 3. 直线前进
    motorStraight(600);     // 60%速度前进
    HAL_Delay(1500);        // 前进1.5秒
    motorStraight(0);       // 停止
    
    HAL_Delay(1000);        // 等待1秒
    
    // 4. 右转90度
    motorTurnRight(1000);   // 右转1秒
    
    HAL_Delay(1000);        // 等待1秒
    
    // 5. 倒车
    motorReverse(500);      // 50%速度倒车
    HAL_Delay(1000);        // 倒车1秒
    motorReverse(0);        // 停止
}

/**
 * @brief 正方形路径运动
 */
void square_path_motion(void)
{
    for(int i = 0; i < 4; i++)
    {
        // 直线前进
        motorStraight(700);     // 70%速度前进
        HAL_Delay(1000);        // 前进1秒
        motorStraight(0);       // 停止
        
        HAL_Delay(500);         // 等待0.5秒
        
        // 左转90度
        motorTurnLeft(1000);    // 左转1秒
        
        HAL_Delay(500);         // 等待0.5秒
    }
}

/**
 * @brief 圆形路径运动
 */
void circle_path_motion(void)
{
    // 通过差速实现圆形运动
    // 左轮速度稍慢，右轮速度稍快
    set_motor_speed(1, 400);    // 左轮40%速度
    set_motor_speed(2, 600);    // 右轮60%速度
    
    HAL_Delay(5000);            // 运动5秒
    
    // 停止
    motor_stop(1);
    motor_stop(2);
}

/**
 * @brief 原地旋转
 */
void spin_motion(void)
{
    // 原地左转360度
    motorTurnLeft(4000);        // 左转4秒
    
    HAL_Delay(1000);            // 等待1秒
    
    // 原地右转360度
    motorTurnRight(4000);       // 右转4秒
}

/**
 * @brief 避障运动模式
 */
void obstacle_avoidance_motion(void)
{
    // 检测到障碍物时的避障动作
    motorStraight(0);           // 立即停止
    
    HAL_Delay(500);             // 等待0.5秒
    
    motorReverse(300);          // 缓慢后退
    HAL_Delay(1000);            // 后退1秒
    motorReverse(0);            // 停止
    
    HAL_Delay(500);             // 等待0.5秒
    
    motorTurnRight(1500);       // 右转1.5秒
    
    HAL_Delay(500);             // 等待0.5秒
    
    motorStraight(600);         // 继续前进
    HAL_Delay(2000);            // 前进2秒
    motorStraight(0);           // 停止
} 
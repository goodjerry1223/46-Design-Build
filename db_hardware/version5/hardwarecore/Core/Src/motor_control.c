/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   电机控制函数实现
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
 * @brief 设置电机速度
 * @param motor: 电机编号 (1=电机A, 2=电机B)
 * @param speed: 速度值 (-1000到1000，负值表示反转)
 */
 
 #include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "wirless.h"
#include "encoder.h"
#include "pid.h"

void fakemove(void)
{
		//向前走   L
	motorStraight(400,24.0f);
	HAL_Delay(2900);
	//向左转
	motorTurnLeft();
	HAL_Delay(380);
	//向前走       S
	motorStraight(400,24.0f);
	HAL_Delay(1500);
	//向左转
	motorTurnLeft();
	HAL_Delay(350);
	//向前走       L
	motorStraight(400,24.0f);  
	HAL_Delay(3000);
	//右转
	motorTurnRight();
	HAL_Delay(330);
	//向前走        S
	motorStraight(400,24.0f);
	HAL_Delay(1800);
	//右转
	motorTurnRight();
	HAL_Delay(380);
	//向前走
	motorStraight(400,24.0f);
	HAL_Delay(2400);
	//向左转
	motorTurnLeft();
	HAL_Delay(380);
	//向前走        S
	motorStraight(400,24.0f);
	HAL_Delay(1800);








}

//此函数调用时，将使得小车向前运动，以占空比400 ，速度为20左右  应用pid调控
//同时发出相应的pid调控信息
void Motormove(void)
{

// 变量定义区
  int motor1_speed = 400;
  int motor2_speed = 400;
  
  uint8_t key_last = 1;
  // PID相关变量
  PID_t pidA;
  PID_t pidB;
  float target_speedA = 20.0f; // 初始目标速度
  float target_speedB = 20.0f; // 初始目标速度
  PID_Init(&pidA, 0.73f, 0.007f, 0.0f, 1000.0f, 0.0f); // 这里的参数请根据实际调试
  PID_Init(&pidB, 0.75f, 0.007f, 0.0f, 1000.0f, 0.0f); // 这里的参数请根据实际调试
  
  
  
  //1111
   set_motor_speed(1, motor1_speed);
   set_motor_speed(2, motor2_speed);
   //yanhsi
   HAL_Delay(100);
   
   while(1)
   {
   
    // 读取编码器计数值
    int32_t encoderA_value = EncoderA_GetCount();  // 电机A编码器计数
    int32_t encoderB_value = EncoderB_GetCount();  // 电机B编码器计数
    // 读取编码器速度（圈/秒）
    double speedA = EncoderA_GetSpeed();
    double speedB = EncoderB_GetSpeed();
	
	
	
	
	 set_motor_speed(1,400);  //此处有一定问题
	 set_motor_speed(2,400);  //此处有一定问题
	
	
	float pid_out1 = PID_Incremental_Calc(&pidA, target_speedA, (float)speedA);
	float pid_out2 = PID_Incremental_Calc(&pidB, target_speedB, (float)speedB);
	
	
	//测试用
	//float error = target_speedB - speedB ; 
	
    motor1_speed += (int)pid_out1;
	
	
    if (pid_out1 > 1000) 
	{pid_out1 = 1000;}
    if (pid_out1 < 0) 
	{pid_out1 = 0;}
	
	motor2_speed += (int)pid_out2;
	
	if (pid_out2 > 1000) 
	{pid_out2 = 1000;}
    if (pid_out2 < 0) 
	{pid_out2 = 0;}
	
	
   set_motor_speed(1, motor1_speed);
    set_motor_speed(2, motor2_speed);
	
	HAL_Delay(100);
    extern UART_HandleTypeDef huart3;
    char buf[128];
    snprintf(buf, sizeof(buf), "%ld,%ld,%.3f,%.3f\n", motor1_speed, motor2_speed, speedA, speedB);
    Wirless_SendString(&huart3, buf);
   
   
   
   
   }



}













 
 
 
 
 
void set_motor_speed(uint8_t motor, int16_t speed)
{
    // 限制速度范围
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;
    
    if (motor == 1) // 电机A (PA6, PA7)
    {
        if (speed < 0) // 正转
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -speed);  // PA6 PWM
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);      // PA7 低电平
        }
        else if (speed > 0) // 反转
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);      // PA6 低电平
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed); // PA7 PWM
        }
        else // 停止
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        }
    }
    else if (motor == 2) // 电机B (PB0, PB1)
    {
        if (speed > 0) // 正转
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);  // PB0 PWM
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);      // PB1 低电平
        }
        else if (speed < 0) // 反转
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);      // PB0 低电平
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -speed); // PB1 PWM
        }
        else // 停止
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        }
    }
}

/**
 * @brief 电机正转
 * @param motor: 电机编号 (1=电机A, 2=电机B)
 */
void motor_forward(uint8_t motor)
{
    set_motor_speed(motor, 500); // 默认50%速度正转
}

/**
 * @brief 电机反转
 * @param motor: 电机编号 (1=电机A, 2=电机B)
 */
void motor_backward(uint8_t motor)
{
    set_motor_speed(motor, -500); // 默认50%速度反转
}

/**
 * @brief 电机停止
 * @param motor: 电机编号 (1=电机A, 2=电机B)
 */
void motor_stop(uint8_t motor)
{
    set_motor_speed(motor, 0);
}

/**
 * @brief 设置电机A速度
 * @param speed: 速度值 (-1000到1000)
 */
void set_motorA_speed(int16_t speed)
{
    set_motor_speed(1, speed);
}

/**
 * @brief 设置电机B速度
 * @param speed: 速度值 (-1000到1000)
 */
void set_motorB_speed(int16_t speed)
{
    set_motor_speed(2, speed);
}

/**
 * @brief 电机A正转
 */
void motorA_forward(void)
{
    set_motor_speed(1, 500);
}

/**
 * @brief 电机A反转
 */
void motorA_backward(void)
{
    set_motor_speed(1, -500);
}

/**
 * @brief 电机A停止
 */
void motorA_stop(void)
{
    set_motor_speed(1, 0);
}
/**
 * @brief 电机AB停止
 */
void motorAB_stop(void)
{
    set_motor_speed(1, 0);
	set_motor_speed(2, 0);
}

/**
 * @brief 电机B正转
 */
void motorB_forward(void)
{
    set_motor_speed(2, 500);
}

/**
 * @brief 电机B反转
 */
void motorB_backward(void)
{
    set_motor_speed(2, -500);
}

/**
 * @brief 电机B停止
 */
void motorB_stop(void)
{
    set_motor_speed(2, 0);
}

/**
 * @brief 直线前进
 * @param speed: 速度值 (-1000到1000，负值表示后退)
 */















void motorStraight(int16_t speed,float target_speed)
{
    // 同时设置两个电机以相同速度运动
    motorAmove(speed, target_speed);  // 电机A
    motorBmove(speed, target_speed);  // 电机B
}

/**
 * @brief 倒车
 * @param speed: 速度值 (-1000到1000，负值表示后退)
	规定倒车的速度也为正
 */
void motorReverse(int16_t speed,float target_speed)
{
    // 同时设置两个电机以相同速度运动
    motorAmove(-speed, target_speed);  // 电机A
    motorBmove(-speed, target_speed);  // 电机B
}

/**
 * @brief 左转
 * @param time: 转向时间 (毫秒)
 */
void motorTurnLeft()
{
    // 左转：电机A后退，电机B前进
	motorAmove(-400, 20.0f);  // 电机A 
	motorBmove(350,18.0f);  // 电机B 
    
    // 延时指定时间
//    HAL_Delay(time);
    
    // 转向完成后停止
//    motor_stop(1);
//    motor_stop(2);
}

/**
 * @brief 右转
 * @param time: 转向时间 (毫秒)
 */
void motorTurnRight()
{
    // 右转：电机A前进，电机B后退
   motorAmove(350, 18.0f);  // 电机A 
   motorBmove(-400,20.0f);  // 电机B 
    
    // 延时指定时间
//    HAL_Delay(time);
    
    // 转向完成后停止
//    motor_stop(1);
//    motor_stop(2);
}





void motorAmove(int16_t speed1,float target_speed)
{
	set_motor_speed(1, speed1);
  // 读取编码器计数值
	//int32_t encoderA_value = EncoderA_GetCount();  // 电机A编码器计数
	double speedA = EncoderA_GetSpeed();
	set_motor_speed(1,400);//风筝
	
	PID_t pidA;
	float target_speedA = target_speed - 0.5f; // 初始目标速度，需要根据实际情况调节*****
	PID_Init(&pidA, 1.2f, 0.13f, 0.0f, 1000.0f, -1000.0f);
	
	float pid_out1 = PID_Incremental_Calc(&pidA, target_speedA, (float)speedA);
	speed1 += (int)pid_out1;
	if (pid_out1 > 1000) 
	{pid_out1 = 1000;}
    if (pid_out1 < -1000) 
	{pid_out1 = -1000;}
	set_motor_speed(1, speed1 - 20 );

}

void motorBmove (int16_t speed2,float target_speed)
{
	set_motor_speed(2, speed2);
  // 读取编码器计数值
	//int32_t encoderB_value = EncoderB_GetCount();  // 电机B编码器计数
	double speedB = EncoderB_GetSpeed();
	set_motor_speed(2,400);//风筝
	
	PID_t pidB;
	float target_speedB = target_speed  ; // 初始目标速度，需要根据实际情况调节*****
	PID_Init(&pidB, 1.3f, 0.13f, 0.0f, 1000.0f, -1000.0f);
	
	float pid_out2 = PID_Incremental_Calc(&pidB, target_speedB, (float)speedB);
	speed2 += (int)pid_out2;
	if (pid_out2 > 1000) 
	{pid_out2 = 1000;}
    if (pid_out2 < -1000) 
	{pid_out2 = -1000;}
	set_motor_speed(2, speed2 );

}



















/* USER CODE END 1 */ 


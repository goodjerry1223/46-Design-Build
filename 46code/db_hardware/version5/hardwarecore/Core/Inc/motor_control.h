/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_control.h
  * @brief   电机控制函数头文件
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

	
	void fakemove(void);
/**
 * @brief 设置电机速度
 * @param motor: 电机编号 (1=电机A, 2=电机B)
 * @param speed: 速度值 (-1000到1000，负值表示反转)
 */
void set_motor_speed(uint8_t motor, int16_t speed);

/**
 * @brief 电机正转
 * @param motor: 电机编号 (1=电机A, 2=电机B)
 */
void motor_forward(uint8_t motor);

/**
 * @brief 电机反转
 * @param motor: 电机编号 (1=电机A, 2=电机B)
 */
void motor_backward(uint8_t motor);

/**
 * @brief 电机停止
 * @param motor: 电机编号 (1=电机A, 2=电机B)
 */
void motor_stop(uint8_t motor);

/**
 * @brief 设置电机A速度
 * @param speed: 速度值 (-1000到1000)
 */
void set_motorA_speed(int16_t speed);

/**
 * @brief 设置电机B速度
 * @param speed: 速度值 (-1000到1000)
 */
void set_motorB_speed(int16_t speed);

/**
 * @brief 电机A正转
 */
void motorA_forward(void);

/**
 * @brief 电机A反转
 */
void motorA_backward(void);

/**
 * @brief 电机A停止
 */
void motorA_stop(void);

/**
 * @brief 电机B正转
 */
void motorB_forward(void);

/**
 * @brief 电机B反转
 */
void motorB_backward(void);

/**
 * @brief 电机B停止
 */
 /**
 * @brief 电机AB停止
 */
void motorAB_stop(void);
void motorB_stop(void);

/**
 * @brief 直线前进
 * @param speed: 速度值 (-1000到1000，负值表示后退)
 */
void motorStraight(int16_t speed,float target_speed);
/**
 * @brief 倒车
 * @param speed: 速度值 (-1000到1000，负值表示后退)
	规定倒车的速度也为正
 */
void motorReverse(int16_t speed,float target_speed);

/**
 * @brief 左转
 * @param time: 转向时间 (毫秒)
 */
void motorTurnLeft();

/**
 * @brief 右转
 * @param time: 转向时间 (毫秒)
 */
void motorTurnRight();



//计划套用pid后的运动控制
//
//
//
//
//
//


/**
 * @brief 使用pid使其A电机以恒定的速度正转
 * @param SPEED 占空比	也可以设置速度
 */
void motorAmove(int16_t speedA,float target_speed);


/**
 * @brief 使用pid使其B电机以恒定的速度正转
 * @param SPEED 占空比    也可以设置速度
 */
void motorBmove (int16_t speedB,float target_speed);














/* USER CODE END Prototypes */











#ifdef __cplusplus
}
#endif
#endif /* __MOTOR_CONTROL_H__ */ 



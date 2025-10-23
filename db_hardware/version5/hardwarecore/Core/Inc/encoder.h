#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "tim.h"

// 读取电机A（TIM2）速度，单位：圈/秒
double EncoderA_GetSpeed(void);
// 读取电机B（TIM4）速度，单位：圈/秒
double EncoderB_GetSpeed(void);
// 读取TIM2当前计数值
uint32_t EncoderA_GetCount(void);
// 读取TIM4当前计数值
uint32_t EncoderB_GetCount(void);
// 编码器定时清零（建议在定时中断里调用）
void Encoder_Reset(void);

#endif // __ENCODER_H__ 

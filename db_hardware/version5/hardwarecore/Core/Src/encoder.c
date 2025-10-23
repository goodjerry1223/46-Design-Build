#include "encoder.h"

// MC520编码器参数
#define ENCODER_PULSE_PER_TURN 390.0 // 输出轴一圈脉冲数（13*30）
#define ENCODER_INTERVAL_MS 50       // 速度采样周期，单位ms

static int32_t last_count_a = 0;
static int32_t last_count_b = 0;

// 读取TIM2当前计数值
uint32_t EncoderA_GetCount(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);
}
// 读取TIM4当前计数值
uint32_t EncoderB_GetCount(void) {
    return __HAL_TIM_GET_COUNTER(&htim4);
}

// 速度计算，单位：圈/秒
double EncoderA_GetSpeed(void) {
    int32_t now = __HAL_TIM_GET_COUNTER(&htim2);
    int32_t diff = now - last_count_a;
    last_count_a = now;
    // 处理定时器溢出
    if (diff > 0x7FFFFFFF) diff -= 0xFFFFFFFF;
    else if (diff < (int32_t)0x80000000) diff += 0xFFFFFFFF;
    double turns = (double)diff / ENCODER_PULSE_PER_TURN;
    return turns / (ENCODER_INTERVAL_MS / 1000.0);
}
double EncoderB_GetSpeed(void) {
    int32_t now = __HAL_TIM_GET_COUNTER(&htim4);
    int32_t diff = now - last_count_b;
    last_count_b = now;
    if (diff > 0x7FFFFFFF) diff -= 0xFFFFFFFF;
    else if (diff < (int32_t)0x80000000) diff += 0xFFFFFFFF;
    double turns = (double)diff / ENCODER_PULSE_PER_TURN;
    return turns / (ENCODER_INTERVAL_MS / 1000.0);
}

// 清零计数（可在定时中断里调用）
void Encoder_Reset(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    last_count_a = 0;
    last_count_b = 0;
} 
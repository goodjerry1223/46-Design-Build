#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

// 增量式PID结构体
typedef struct {
    float Kp;      // 比例系数
    float Ki;      // 积分系数
    float Kd;      // 微分系数
    float prev_error;    // 上一次误差
    float prev_prev_error; // 上上次误差
    float output;  // 当前输出
    float max_output; // 输出上限
    float min_output; // 输出下限
} PID_t;

// PID初始化
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float max_output, float min_output);

// 增量式PID计算
float PID_Incremental_Calc(PID_t *pid, float setpoint, float measured);

// 可选：重置PID
void PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif // PID_H 

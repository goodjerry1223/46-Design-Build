#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float max_output, float min_output) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output = 0.0f;
    pid->max_output = max_output;
    pid->min_output = min_output;
}

float PID_Incremental_Calc(PID_t *pid, float setpoint, float measured) {
    float error = setpoint - measured;
    float delta_output = pid->Kp * (error - pid->prev_error)
                        + pid->Ki * error
                        + pid->Kd * (error - 2 * pid->prev_error + pid->prev_prev_error);
    pid->output += delta_output;
    // 限幅
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < pid->min_output) pid->output = pid->min_output;
    // 更新误差
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    return pid->output;
}

void PID_Reset(PID_t *pid) {
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output = 0.0f;
} 
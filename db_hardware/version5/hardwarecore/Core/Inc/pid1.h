#ifndef PID1_H
#define PID1_H

#include "main.h"
#include "pid1.h"

typedef struct
{
    float setpoint;             //目标值
    float actualvalue;          //期望值
    float sumerror;             //累计偏差
    float P;
    float I;
    float D;
    float error;                //error[K]
    float lasterror;            //error[K-1]
    float preverror;            //error[K-2]

}PID_controller;

void pid_init_yaw_v(void);

float increment__yaw_v(PID_controller *PID,float Feedback_value);

void Change_yaw_v_setpoint(PID_controller *PID,float Feedback_value);

#endif

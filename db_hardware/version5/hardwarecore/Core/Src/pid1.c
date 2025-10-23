#include "pid1.h"

PID_controller  yaw_v_pid;
extern float PID_yaw_v[];
extern float Mechanical_zero[];


void pid_init_yaw_v(void)
{
	  yaw_v_pid.setpoint=Mechanical_zero[1];
    yaw_v_pid.actualvalue=0;
    yaw_v_pid.sumerror=0;
    yaw_v_pid.error=0;
    yaw_v_pid.lasterror=0;
    yaw_v_pid.preverror=0;
    yaw_v_pid.P=PID_yaw_v[0];
    yaw_v_pid.I=PID_yaw_v[1];
    yaw_v_pid.D=PID_yaw_v[2];
}

float increment__yaw_v(PID_controller *PID,float Feedback_value)
{
	  PID->error=(float)(PID->setpoint-Feedback_value);
    PID->sumerror +=PID->error;
    PID->actualvalue =(PID->P*PID->error)
                     +(PID->I*PID->sumerror)
                     +(PID->D*(PID->error-PID->lasterror));
    PID->lasterror=PID->error;
    return PID->actualvalue;
}

void Change_yaw_v_setpoint(PID_controller *PID,float setpoint)
{
		PID->setpoint=setpoint;
}


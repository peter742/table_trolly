#include "PID.h"

_pid pid;

void PID_param_init()
{  
	/* 初始化参数 */	
	 	UsartPrintf(USART_DEBUG,"PID_init begin \n");
		 pid.target_val=0.0;
		 pid.actual_val=0.0;
		 pid.err=0.0;
		 pid.err_last=0.0;
		 pid.integral=0.0;
		 pid.Kp = 0.31;
		 pid.Ki = 0.070;
		 pid.Kd = 0.3;
		 UsartPrintf(USART_DEBUG,"PID_init end \n");
}

float PID_realize(float temp_val)
{  
		 /* 计算目标值与实际值的误差 */
		 pid.err=pid.target_val-temp_val;
		 /* 误差累积 */
		 pid.integral+=pid.err;
		 /*PID 算法实现 */
		 pid.actual_val=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
		 /* 误差传递 */
		 pid.err_last=pid.err;
		 /* 返回当前实际值 */
		 return pid.actual_val;
}
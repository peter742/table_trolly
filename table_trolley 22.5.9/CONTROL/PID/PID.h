#ifndef _PID_H_
#define _PID_H_

#include "main.h"
#include "usart.h"

typedef struct
{
		float target_val; //目标值
		float actual_val; //实际值
		float err; //定义偏差值
		float err_last; //定义上一个偏差值
		float Kp,Ki,Kd; //定义比例、积分、微分系数
		float integral; //定义积分值
}_pid;




void PID_param_init();











#endif

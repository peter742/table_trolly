#ifndef _PID_H_
#define _PID_H_

#include "main.h"
#include "usart.h"

typedef struct
{
		float target_val; //Ŀ��ֵ
		float actual_val; //ʵ��ֵ
		float err; //����ƫ��ֵ
		float err_last; //������һ��ƫ��ֵ
		float Kp,Ki,Kd; //������������֡�΢��ϵ��
		float integral; //�������ֵ
}_pid;




void PID_param_init();











#endif

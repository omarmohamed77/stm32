/*
 * Servo.c
 *
 *  Created on: Nov 17, 2020
 *      Author: omar
 */
#include "Servo.h"
void Servo_Init(void){
Timer_PWMInit();
}
void Servo_SetPostion(Servo_Postion position){
	switch (position){
	case POS_DEG_0:
		Timer_PWM_SetDuty(7);
		break;
	case POS_DEG_90:
		Timer_PWM_SetDuty(10);
		break;
	case POS_DEG_180:
		Timer_PWM_SetDuty(5);
		break;
	default:
		break;
	}
	delayMs(600);
	Timer_PWM_SetDuty(0);
}


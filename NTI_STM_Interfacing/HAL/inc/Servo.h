/*
 * Servo.h
 *
 *  Created on: Nov 17, 2020
 *      Author: omar
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_
#include "timer.h"
#include "delay.h"

#include "../../Device/STM32F4xx/Include/stm32f4xx.h"
typedef enum{
	POS_DEG_0,POS_DEG_90,POS_DEG_180

}Servo_Postion;

void Servo_Init(void);
void Servo_SetPostion(Servo_Postion position);


#endif /* INC_SERVO_H_ */

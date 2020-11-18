/*
 * timer.h
 *
 *  Created on: Nov 15, 2020
 *      Author: omar
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_
#include "../../Device/STM32F4xx/Include/stm32f4xx.h"
/* Time Management*/
void Timer_DelaysMS(uint32_t delay_ms);
void Timer_DelayUS(uint32_t);
void Timer_BaseInit(void);


/*
 * input capture
 */
void Timer_CaptureInit(void);

/*PWM MODE OUT COMPARE*/
void Timer_PWMInit(void);
void Timer_PWM_SetDuty(uint8_t duty);
void Timer_PWM_IncDuty(uint8_t incDuty);
void Timer_PWM_DecDuty(uint8_t incDuty);
void Timer_PWM_LED_Init(void);
void Timer_PWM_Set_LED_Duty(uint8_t duty);

#endif /* INC_TIMER_H_ */

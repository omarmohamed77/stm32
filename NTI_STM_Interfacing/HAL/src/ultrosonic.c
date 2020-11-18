/*
 * ultrosonic.c
 *
 *  Created on: Nov 16, 2020
 *      Author: omar
 */
#include "ultrasonic.h"

extern volatile uint32_t pulse_width;

void	Ultra_Init(void){
	/*setup trigger*/
	RCC->AHB1ENR		|= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER		&= ~(3<<(4*2));
	GPIOA->MODER		|= (1<<(4*2));
	GPIOA->ODR			&= ~GPIO_ODR_ODR_4;

	Timer_CaptureInit();
}
uint16_t	Ultra_GetDistance(void){
	uint32_t delay = 0; /* in Ms*/
	pulse_width =0;
	uint16_t distance =0;
	/*
	 Send the trigger puls*/
	GPIOA->ODR			|= GPIO_ODR_ODR_4;
	delayMs(1);
	GPIOA->ODR			&= ~GPIO_ODR_ODR_4;
	 /*wait for pulse width measurement to be completed*/

	while(pulse_width == 0 && delay < 30){
		delayMs(1);
		delay++;
	}
	/* calculate distance where distance = width/58*/
	distance = pulse_width/58;
	/* return distance*/
	return distance;
}

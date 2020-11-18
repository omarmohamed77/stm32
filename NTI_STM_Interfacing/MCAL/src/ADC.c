/*
 * ADC.c
 *
 *  Created on: Nov 11, 2020
 *      Author: omar
 */
#include "ADC.h"
void ADC1_Init(void){

	RCC->APB2ENR      |= RCC_APB2ENR_ADC1EN;

	ADC1->CR2         |= ADC_CR2_ADON;

	ADC1->SQR1        &= ~ ADC_SQR1_L;

}

void ADC1_SelectChannel(ADC_CH channel){
	switch (channel) {
	case CH0:
		/*PA0*/
		RCC->AHB1ENR   |= RCC_AHB1ENR_GPIOAEN;
		GPIOA->MODER   |= GPIO_MODER_MODER0;
		break;
	case CH1:
		/*PA1*/
		RCC->AHB1ENR   |= RCC_AHB1ENR_GPIOAEN;
		GPIOA->MODER   |= GPIO_MODER_MODER1;
		break;
	default:
		break;

		}
	ADC1->SQR3        &= ~ADC_SQR3_SQ1;
	ADC1->SQR3        |= channel;


}

uint16_t ADC1_Read(void){

	ADC1->CR2        |= ADC_CR2_SWSTART;

	while( !(ADC1->SR &ADC_SR_EOC) );

	return ADC1->DR;
}

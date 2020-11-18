/*
 * timer.c
 *
 *  Created on: Nov 15, 2020
 *      Author: omar
 */
#include"timer.h"
/*timer 2 for time management
 * timer 1 ch1  IC for ultrasonic echo
 * timer 4 ch OC-PWM   for servo
 */
void Timer_BaseInit(void){
	/* RCC*/
	RCC->APB1ENR	|= RCC_APB1ENR_TIM2EN;



	/* Timer PSC */
	TIM2->PSC		=16-1;

	/* Enable CEN */
	TIM2->CR1		|=TIM_CR1_CEN;
}
void Timer_DelaysMS(uint32_t delay_ms){
	TIM2->CNT		=0;
	TIM2->ARR		= delay_ms*1000;
	//while(TIM2->CNT < TIM2->ARR );
	while( ! (TIM2->SR & TIM_SR_UIF));
	TIM2->SR &= ~TIM_SR_UIF;

}
void Timer_DelayUS(uint32_t delay_us){
	TIM2->CNT		=0;
	TIM2->ARR		= delay_us;
	while(TIM2->CNT < TIM2->ARR );




}

void Timer_CaptureInit(void){
	/*GPIOA PA8 (RCC, TIM1 alterate)*/
	RCC->AHB1ENR	|=RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER	&= ~(3<<(8*2));
	GPIOA->MODER	|= (2<<(8*2));
	GPIOA->AFR[1]	&= ~(0xF<<(0*4));
	GPIOA->AFR[1]	|= (0x1<<(0*4));
	/*TIM1 RCC (APB2)*/
	RCC->APB2ENR	|= RCC_APB2ENR_TIM1EN;

	/*TIM1 tick */
	TIM1->PSC		= 16 - 1;

	/*ARR max*/
	TIM1->ARR		=0XFFFFFFFF;

	/*CH1 is input IC1 */
	TIM1->CCMR1		|= (TIM_CCMR1_CC1S_0);
	TIM1->CCMR1		&= ~(TIM_CCMR1_CC1S_1);

	/* select rising and falling edges */
	TIM1->CCER		|= TIM_CCER_CC1P;
	TIM1->CCER		|= TIM_CCER_CC1NP;

	/*Enable :channel*/
	TIM1->CCER		|= TIM_CCER_CC1E;

	/*Enable timer*/
	TIM1->CR1		|= TIM_CR1_CEN;

	/* capture /compare interrupt */
	TIM1->DIER		|= TIM_DIER_CC1IE;

	/*capture /compare interrupt nvic enable and global */

	NVIC_EnableIRQ(TIM1_CC_IRQn);
	__enable_irq();

}
volatile uint32_t pulse_width =0;
static volatile uint32_t edge_state =0; /* 0 : rising, 1: Falling*/

void TIM1_CC_IRQHandler(void){
	TIM1->SR	&=~TIM_SR_CC1IF;
	if(edge_state ==0){
		TIM1->CNT	= 0;
		edge_state =1;
	}else{
		pulse_width = TIM1->CCR1;
		edge_state = 0;
	}

	}
void Timer_PWMInit(void){
	/*GPIOB PB9*/
	RCC->AHB1ENR	|=RCC_AHB1ENR_GPIOBEN;
		GPIOB->MODER	&= ~(3<<(9*2));
		GPIOB->MODER	|= (2<<(9*2));

		GPIOB->AFR[1]	&= ~(0xF<<(1*4));
		GPIOB->AFR[1]	|= (0x2<<(1*4));
		/*TIM1 RCC (APB1)*/
		RCC->APB1ENR	|= RCC_APB1ENR_TIM4EN;

		/*TIM1 tick */
		TIM4->PSC		= (SystemCoreClock/1000000UL) - 1;

		/*ARR max*/
		TIM4->ARR		=20000;

		/*CH4 is output oc4 */
		TIM4->CCMR2		&= ~(TIM_CCMR2_CC4S);

		/* select pwm mode (CCMR2_OC4M :110) */
		TIM4->CCMR2		|= (TIM_CCMR2_OC4M);
		TIM4->CCMR2		&= ~(TIM_CCMR2_OC4M_0);
		/*preload enable*/
		TIM4->CCMR2		|= (TIM_CCMR2_OC4PE);
		TIM4->CR1		|= (TIM_CR1_ARPE);

		/* Enable SET update*/
		TIM4->EGR		|= TIM_EGR_UG;

		/*Main output enable (BDTR_MOE)*/
		TIM4->BDTR		|= TIM_BDTR_MOE;

		/*Enable :channel*/
		TIM4->CCER		|= TIM_CCER_CC4E;

		/*Enable timer*/
		TIM4->CR1		|= TIM_CR1_CEN;


}
void Timer_PWM_SetDuty(uint8_t duty){
	TIM4->CCR4			=(TIM4->ARR) *(duty/100.0);
}
void Timer_PWMIncDuty(uint8_t incDuty){
	if( (incDuty + ((100*TIM4->CCR4)/TIM4->ARR)) < 100)
	{
		TIM4->CCR4	+= ((TIM4->ARR)*(incDuty/100.0));
	}
	else{
		TIM4->CCR4   = TIM4->ARR;
	}
}
void Timer_PWMDecDuty(uint8_t decDuty){
	if( ((100*TIM4->CCR4)/TIM4->ARR) > decDuty )
	{
		TIM4->CCR4	-= ((TIM4->ARR)*(decDuty/100.0));
	}
	else{
		TIM4->CCR4   = 0;
	}
}
void Timer_PWM_LED_Init(void){
	/*GPIOc PB6*/
	RCC->AHB1ENR	|=RCC_AHB1ENR_GPIOCEN;
		GPIOC->MODER	&= ~(3<<(6*2));
		GPIOC->MODER	|= (2<<(6*2));

		GPIOC->AFR[0]	&= ~(0xF<<(6*4));
		GPIOC->AFR[0]	|= (0x2<<(6*4));
		/*TIM1 RCC (APB1)*/
		RCC->APB1ENR	|= RCC_APB1ENR_TIM3EN;

		/*TIM1 tick */
		TIM3->PSC		= (SystemCoreClock/1000UL) - 1;

		/*ARR max*/
		TIM3->ARR		=20;

		/*CH4 is output oc4 */
		TIM3->CCMR1		&= ~(TIM_CCMR1_CC1S);

		/* select pwm mode (CCMR2_OC4M :110) */
		TIM3->CCMR1		|= (TIM_CCMR1_OC1M);
		TIM3->CCMR1		&= ~(TIM_CCMR1_OC1M_0);
		/*preload enable*/
		TIM3->CCMR1		|= (TIM_CCMR1_OC1PE);
		TIM3->CR1		|= (TIM_CR1_ARPE);

		/* Enable SET update*/
		TIM3->EGR		|= TIM_EGR_UG;

		/*Main output enable (BDTR_MOE)*/
		TIM3->BDTR		|= TIM_BDTR_MOE;

		/*Enable :channel*/
		TIM3->CCER		|= TIM_CCER_CC1E;

		/*Enable timer*/
		TIM3->CR1		|= TIM_CR1_CEN;


}
void Timer_PWM_Set_LED_Duty(uint8_t duty){
	TIM3->CCR1			=(TIM3->ARR) *(duty/100.0);
}

/*
 * uart.c
 *
 *  Created on: Nov 18, 2020
 *      Author: omar
 */

#include "uart.h"
void Uart_Init(uint32_t baudrate){
	uint8_t frac;
	uint16_t integ;
	float UARTDIV;

	RCC->AHB1ENR	|=RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER	&= ~(3<<(2*2));
	GPIOA->MODER	&= ~(3<<(3*2));
	GPIOA->MODER	|= (2<<(2*2));
	GPIOA->MODER	|= (2<<(3*2));

	GPIOA->AFR[0]	&= ~(0xF<<(2*4));
	GPIOA->AFR[0]	&= ~(0xF<<(3*4));
	GPIOA->AFR[0]	|= (0x7<<(2*4));
	GPIOA->AFR[0]	|= (0x7<<(3*4));
	RCC->APB1ENR	|= RCC_APB1ENR_USART2EN;

	UARTDIV			= (SystemCoreClock/(16.0*baudrate));
	integ			= (uint16_t)UARTDIV;
	frac			= (UARTDIV-integ)*16;
	USART2->BRR		=(integ<<4)+frac;

	USART2->CR1		|=((USART_CR1_TE)|(USART_CR1_RE)|(USART_CR1_UE));


}
void Uart_SendByte(uint8_t byte){
	while(!(USART2->SR &USART_SR_TXE));
	USART2->DR 		=byte;
}
void Uart_SendString(uint8_t* str){
	uint32_t i		=0;
	while(str[i]){
		Uart_SendByte(str[i]);
		i++;

	}
}
void Uart_SendBytes(uint8_t* buf,uint32_t len){

	for(uint32_t i=0;i<len;i++){

		Uart_SendByte(buf[i]);
	}
}

void Uart_ReceiveByte(uint8_t* pData){
	while (!(USART2->SR &USART_SR_RXNE));
	*pData = USART2->DR;
}
uint8_t Uart_ReceiveByte_Unblock(uint8_t* pData){
	uint8_t status	= 0;
	if((USART2->SR &USART_SR_RXNE)){
		(*pData)	= USART2->DR;
		status		= 1;
	}
	return status;
}
/*Timeout in ms */
uint8_t Uart_ReceiveBytes(uint8_t* buf,uint32_t len,uint32_t timeout){
	uint8_t i	= 0;
	uint8_t status =0;
	uint32_t counter	= 0;

	while (i<len && counter <timeout){
		if(Uart_ReceiveByte_Unblock(buf+i)){
			i++;

		}
		else{
			counter++;
			delayMs(1);

		}
	}
	if(counter< timeout){

		status = 1;
	}
	else{
		status =0;
	}
	return status;
}
uint8_t Uart_ReceiveBytes_Del(uint8_t* buf,uint8_t delimeter,uint32_t* plen,uint32_t timeout){
	uint8_t i			= 0;
	uint8_t status		= 0;
	uint32_t counter	= 0;
	uint8_t data		= delimeter+1;

		while (data != delimeter && counter < timeout){
			if(Uart_ReceiveByte_Unblock(buf+i)){
				data = buf[i];
				i++;

			}
			else{
				counter++;
				delayMs(1);

			}
		}

		if(counter< timeout){

			status = 1;
		}
		else{
			status =0;
		}
		return status;
}


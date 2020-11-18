/*
 * uart.h
 *
 *  Created on: Nov 18, 2020
 *      Author: omar
 */

#ifndef INC_UART_H_
#define INC_UART_H_
#include "timer.h"
#include "delay.h"
#include "stm32f4xx.h"
/*UART 2*/
void Uart_Init(uint32_t baudrate);
void Uart_SendByte(uint8_t byte);
void Uart_SendString(uint8_t* str);
void Uart_SendBytes(uint8_t* buf,uint32_t len);

void Uart_ReceiveByte(uint8_t* pData);
uint8_t Uart_ReceiveByte_Unblock(uint8_t* pData);
/*Timeout in ms */
uint8_t Uart_ReceiveBytes(uint8_t* buf,uint32_t len,uint32_t timeout);
uint8_t Uart_ReceiveBytes_Del(uint8_t* buf,uint8_t delimeter,uint32_t* plen,uint32_t timeout);



#endif /* INC_UART_H_ */

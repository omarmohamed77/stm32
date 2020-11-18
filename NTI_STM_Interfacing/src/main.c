/*
 ******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 9.3.0   2020-11-04

The MIT License (MIT)
Copyright (c) 2019 STMicroelectronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 ******************************************************************************
 */

/* Includes */
#include "delay.h"
#include "board.h"
#include "seven_seg.h"
#include "keypad.h"
#include "lcd.h"
#include "ADC.h"
#include "ultrasonic.h"
#include "Servo.h"
#include"uart.h"

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

/**
 **===========================================================================
 **
 **  Abstract: main program
 **
 **===========================================================================
 */
void BTN_Center_Handler(void){

	Leds_Toggle(0xFF);
}
void BTN_Down_Handler(void){

	Leds_Toggle(0xFF);
}
void BTN_Left_Handler(void){

	Leds_Toggle(0xFF);
}
void BTN_UP_Handler(void){

	Leds_Toggle(0xFF);
}
void BTN_Right_Handler(void){

	Leds_Toggle(0xFF);
}


int main(void)
{





	uint8_t key = 0;
	/* Clock -> Internal 16 MHz */
	RCC_DeInit();			/* Adapt PLL to the internal 16 MHz RC oscillator */
	SystemCoreClockUpdate();	/* Update SystemCoreClock */
	uint8_t data	= 0;
	uint8_t buf[100];
	uint32_t len =0;


	Uart_Init(9600);
	if(Uart_ReceiveBytes_Del(buf,'\n',&len ,10000)){
		Uart_SendString("\nRX data: \n");
		Uart_SendBytes(buf,5);
	}else{
			Uart_SendString("\n timeout\n");

		}
	while(1){
		//if(Uart_ReceiveByte_Unblock(&data)){
		//Uart_SendByte(data+1);
		}


	//asm("movs sp,#0x20017000 ");
	//asm("");
	/*asm("const1: .word 0x2001700F");
	asm("LDR R0,=const1");
	asm("LDR R0,[R0]");
	asm("MOV SP,R0 ");*/
	uint16_t adcValue;
	uint8_t temp;
	LCD_Init();
	Timer_PWM_LED_Init();	//Timer_PWMInit();
	//LCD_DispStrXY(0,0,"Servo Test");

	while(1){
		Timer_PWM_Set_LED_Duty(10);
		delayMs(400);
		Timer_PWM_Set_LED_Duty(20);
		delayMs(400);
		Timer_PWM_Set_LED_Duty(40);
		delayMs(400);
		Timer_PWM_Set_LED_Duty(60);
		delayMs(400);
		Timer_PWM_Set_LED_Duty(80);
		delayMs(400);
		Timer_PWM_Set_LED_Duty(100);
	}

	//Btn_Init_EXTI(BTN_CENTER,BTN_Center_Handler);
	//Btn_Init_EXTI(BTN_DOWN,BTN_Center_Handler);
	//Btn_Init_EXTI(BTN_LEFT,BTN_Center_Handler);
	//Btn_Init_EXTI(BTN_RIGHT,BTN_Center_Handler);
	//Btn_Init_EXTI(BTN_UP,BTN_Center_Handler);

	while(1){
		Timer_DelaysMS(1000);
		Leds_Toggle(0xFF);



	}
	while(1){

	adcValue = ADC1_Read();
	temp=(adcValue*(0.8/1000)/0.01);
	//LCD_DispStrXY(1,1,str);
	LCD_DispIntXY(1,1,temp);
	delayMs(100);
	}
/*
	Leds_Off(0xFF);
	Leds_On(1<<0);
	delayMs(1000);
	Leds_Off(0xFF);
	Leds_On(1<<1);
	delayMs(1000);
	Leds_Off(0xFF);
	Leds_On(1<<2);
	delayMs(1000);
	Leds_Off(0xFF);
	Leds_On(1<<3);
	delayMs(1000);
	Leds_Off(0xFF);
	Leds_On(1<<4);
	delayMs(1000);
	Leds_Off(0xFF);
	Leds_On(1<<5);
	delayMs(1000);
	Leds_Off(0xFF);
	Leds_On(1<<6);
	delayMs(1000);
	Leds_Off(0xFF);
	Leds_On(1<<7);
	delayMs(1000);
*/
	while(1){
		key = Keypad_GetKey();
		if(key){
			Leds_Off(0xFF);
			Leds_On(key);
			//delayMs(1000);
		}
	}

	//Segment_Init();
	while(1){
		Segment_Display(2,1);
		Segment_Display(0,2);
		Segment_Display(2,3);
		Segment_Display(1,4);
	}


	Buz_Init();
	Relay_Init();

	Buz_Toggle();
	delayMs(1000);
	Buz_Toggle();
	delayMs(1000);

	Btn_Init(BTN_CENTER);
	Btn_Init(BTN_UP);
	Btn_Init(BTN_DOWN);
	Btn_Init(BTN_LEFT);
	Btn_Init(BTN_RIGHT);
	Leds_Init(ALL_LEDS);
	while (1)
	{
		if(Btn_isPressed(BTN_CENTER)){
			Leds_Toggle(0xFF);
			while(Btn_isPressed(BTN_CENTER));
		}
		if(Btn_isPressed(BTN_RIGHT)){
			Relay_Toggle();
			while(Btn_isPressed(BTN_RIGHT));
		}
		if(Btn_isPressed(BTN_UP)){
			Buz_Toggle();
			Leds_Toggle(0xF0);
			while(Btn_isPressed(BTN_UP));
		}
		if(Btn_isPressed(BTN_LEFT)){
			Leds_Toggle(0xF0);
			while(Btn_isPressed(BTN_LEFT));
		}
		if(Btn_isPressed(BTN_DOWN)){
			Leds_Toggle(0x0F);
			while(Btn_isPressed(BTN_DOWN));
		}
	}
}
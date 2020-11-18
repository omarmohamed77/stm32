/*
 * lcd.h
 *
 *  Created on: Nov 10, 2020
 *      Author: omar
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "delay.h"
#include "string.h"
#include "stdio.h"
#include "../../Device/STM32F4xx/Include/stm32f4xx.h"

#define LCD_RCC                        RCC_AHB1ENR_GPIOBEN
#define LCD_GPIO                       GPIOB
#define LCD_DATA_SHIFT                 12


//#define DATA_MASK      0x1FE0
#define LCD_DATA_ODR_MASK              (0xF<<LCD_DATA_SHIFT)
#define LCD_DATA_MODER_MASK            (0xFF<<(LCD_DATA_SHIFT*2))
#define LCD_DATA_MODER_OUT_MASK        (0x55<<(LCD_DATA_SHIFT*2))

#define LCD_CTRL_EN                    5
#define LCD_CTRL_RS                    8
#define LCD_CTRL_MODER_MASK            ((0X3<<(LCD_CTRL_EN*2))|(0X3<<(LCD_CTRL_RS*2)))
#define LCD_CTRL_MODER_OUT_MASK        ((0X1<<(LCD_CTRL_EN*2))|(0X1<<(LCD_CTRL_RS*2)))

/* Commands*/

#define LCD_CMD_CLEAR                 0X01
#define LCD_CMD_HOME                  0X02
#define LCD_CMD_ENTRY_INC             0X06
#define LCD_CMD_DISP                  0X0C
#define LCD_CMD_FUN_SET               0X28

#define LCD_L1_ADDR                   0X00
#define LCD_L2_ADDR                   0X40
#define LCD_CMD_DDRAM                 0X80
/* */

/*BASIC*/

/*
 *
 * x (col: 0:15)
 * y (line 0:1)
 */



void LCD_Init(void);
void LCD_Clear(void);
void LCD_DispChar(uint8_t data);
void LCD_DispCharXY(uint8_t line,uint8_t col ,uint8_t data);
void LCD_DispStr(uint8_t* str);
void LCD_DispStrXY(uint8_t line,uint8_t col ,uint8_t* str);
void LCD_DispInt(uint32_t num);
void LCD_DispIntXY(uint8_t line,uint8_t col ,uint32_t num);
#endif /* INC_LCD_H_ */

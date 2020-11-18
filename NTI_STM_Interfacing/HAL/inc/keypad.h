/*
 * keypad.h
 *
 *  Created on: Nov 9, 2020
 *      Author: omar
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

#include "delay.h"
#include "../../Device/STM32F4xx/Include/stm32f4xx.h"

#define KEY_ROWS_SHIFT                 5
#define KEY_COLS_SHIFT                 0
#define KEY_ROWS_RCC                   RCC_AHB1ENR_GPIOAEN

#define KEY_ROWS_GPIO                  GPIOA
//#define DATA_MASK      0x1FE0
#define KEY_ROWS_ODR_MASK              (0x7<<KEY_DATA_SHIFT)
#define KEY_ROWS_MODER_MASK            (0x3F<<(KEY_ROWS_SHIFT*2))
#define KEY_ROWS_MODER_OUT_MASK        (0x15<<(KEY_ROWS_SHIFT*2))

#define KEY_COLS_RCC                   RCC_AHB1ENR_GPIOBEN

#define KEY_COLS_GPIO                  GPIOB
//#define DATA_MASK      0x1FE0
//#define KEY_COLS_ODR_MASK              (0x7<<KEY_DATA_SHIFT)
#define KEY_COLS_MODER_MASK            (0x3F<<(KEY_COLS_SHIFT*2))
#define KEY_COLS_PUPDR_MASK            (0x3F<<(KEY_COLS_SHIFT*2))
#define KEY_COLS_PUPDR_UP_MASK        (0x15<<(KEY_COLS_SHIFT*2))

void Keypad_Init(void);
uint8_t Keypad_GetKey(void);
/* key ->1:9
key 1-> R1 C1
*/


#endif /* INC_KEYPAD_H_ */

/*
 * seven_seg.h
 *
 *  Created on: Nov 9, 2020
 *      Author: omar
 */

#ifndef INC_SEVEN_SEG_H_
#define INC_SEVEN_SEG_H_
#include "delay.h"

#include "../../Device/STM32F4xx/Include/stm32f4xx.h"
/* numbers */

#define NUM_0 0X3F
#define NUM_1 0X06
#define NUM_2 0X5B
#define NUM_3 0X4F
#define NUM_4 0X66
#define NUM_5 0X6D
#define NUM_6 0X7D
#define NUM_7 0X07
#define NUM_8 0X7F
#define NUM_9 0X6F
#define DOT 0X80
/*
 *  DATA -> PC5:12 ACTIVE HIGH
 * POS -> PB12:15 ACTIVE LOW
 *  */
#define SEG_DATA_SHIFT                 5
#define SEG_POS_SHIFT                  12
#define SEG_DATA_RCC                   RCC_AHB1ENR_GPIOCEN

#define SEG_DATA_GPIO                  GPIOC
//#define DATA_MASK      0x1FE0
#define SEG_DATA_ODR_MASK              (0xFF<<SEG_DATA_SHIFT)
#define SEG_DATA_MODER_MASK            (0xFFFF<<(SEG_DATA_SHIFT*2))
#define SEG_DATA_MODER_OUT_MASK        (0x5555<<(SEG_DATA_SHIFT*2))

#define SEG_POS_RCC                    RCC_AHB1ENR_GPIOBEN
#define SEG_POS_GPIO                   GPIOB
#define SEG_POS_ODR_MASK               (0xF<<SEG_POS_SHIFT)
#define SEG_POS_MODER_MASK             (0xFF<<(SEG_POS_SHIFT*2))
#define SEG_POS_MODER_OUT_MASK             (0x55<<(SEG_POS_SHIFT*2))


void Segment_Init(void);
void Segment_Display(uint8_t digitValue , uint8_t position);

#endif /* INC_SEVEN_SEG_H_ */

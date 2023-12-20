/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Gyeonheal An
Created          : 05-03-2021
Modified         : 10-14-2023
Language/ver     : C++ in Keil uVision

Description      : SysTick.h
/----------------------------------------------------------------*/

#ifndef __EC_SYSTICK_H
#define __EC_SYSTICK_H

#include "stm32f4xx.h"
#include "ecRCC.h"
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

extern volatile uint32_t msTicks;
void SysTick_init(void);
void SysTick_Handler(void);
void SysTick_counter();
void delay_ms(uint32_t msec);
void SysTick_reset(void);
uint32_t SysTick_val(void);
void SysTick_enable(void);
void SysTick_disable(void);
void delay_us (uint32_t uesc);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

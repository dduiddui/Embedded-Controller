/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Gyeonheal An
Created          : 05-03-2021
Modified         : 10-14-2023
Language/ver     : C++ in Keil uVision

Description      : SysTick.c
/----------------------------------------------------------------*/

#include "ecSysTick.h"

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

volatile uint32_t msTicks=0;
volatile uint32_t usTicks=0;
//EC_SYSTEM_CLK

void SysTick_init(void){	
	//  SysTick Control and Status Register
	SysTick->CTRL = 0;											// Disable SysTick IRQ and SysTick Counter

	// Select processor clock
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

	// uint32_t MCU_CLK=EC_SYSTEM_CLK
	// SysTick Reload Value Register
	SysTick->LOAD = MCU_CLK_PLL / 1000000 - 1;						// 1us, for HSI PLL = 84MHz.

	// SysTick Current Value Register
	SysTick->VAL = 0;

	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	
	// Enable SysTick IRQ and SysTick Timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
		
	NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC
}



void SysTick_Handler(void){
	SysTick_counter();	
}

void SysTick_counter(){
	msTicks++;
	usTicks++;
}	


void delay_ms (uint32_t mesc){
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < mesc);
	
	msTicks = 0;
}


void delay_us (uint32_t usec){
  uint32_t curTicks;

  curTicks = usTicks;
  while ((usTicks - curTicks) < usec);
	
	usTicks = 0;
}
// void delay_ms(uint32_t msec){
//	uint32_t now=SysTick_val(); 
//	if (msec>5000) msec=5000;
//	if (msec<1) msec=1;
//	while ((now - SysTick_val()) < msec);
//}


void SysTick_reset(void)
{
	// SysTick Current Value Register
	SysTick->VAL = 0;
}

uint32_t SysTick_val(void) {
	return SysTick->VAL;
}

//void SysTick_counter(){
//	msTicks++;
//	if(msTicks%1000 == 0) count++;
//}	

void SysTick_enable(void) {
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void SysTick_disable(void) {
	SysTick->CTRL = 0;
}
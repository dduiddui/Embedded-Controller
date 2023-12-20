/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : DUWON YANG
Created          : 05-03-2021
Modified         : 10-13-2023
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_EXTI_SysTick
/----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"

#include "ecSysTick.h"

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

//volatile uint32_t msTicks = 0;
//volatile uint32_t curTicks;

void setup(void);
void LED_Toggle(void);
void EXTI15_10_IRQHandler(void);
unsigned int cnt = 0;

int main(void) {
	
// System Initialiization ----------------------------------------
	setup();
		
// While loop ------------------------------------------------------				
	msTicks = 0;

	while(1){

		sevensegment_display(cnt % 10);
		delay_ms(1000);	
		cnt++;
		if(cnt > 9) cnt = 0;
		SysTick_reset();
	}
	
}


void setup(void)
{
	// System CLOCK, GPIO Initialiization ----------------------------------------
	RCC_PLL_init();                         // System Clock = 84MHz
	// Initialize GPIOA_5 for Output
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOA, LED_PIN, EC_PUSH_PULL); 
	GPIO_pupd(GPIOA, LED_PIN, EC_NONE);
	// Initialize GPIOC_13 for Input Button
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_otype(GPIOC, BUTTON_PIN, EC_PUSH_PULL);

	
	// EXTI Initialization ------------------------------------------------------	
	EXTI_init(GPIOC, BUTTON_PIN, FALL,0);
	
	// sevensegment Initialization-----------------------------------------------
	sevensegment_display_init();
	
	SysTick_init();
}

void LED_Toggle(void){

				GPIOA->ODR ^= (1UL << LED_PIN);	 		
}

void EXTI15_10_IRQHandler(void){
	if (is_pending_EXTI(BUTTON_PIN)){
			cnt = 0;
			clear_pending_EXTI(BUTTON_PIN);
		}
	
}

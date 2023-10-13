/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM
* @brief   Embedded Controller:  Tutorial ___
*					 - _________________________________
*
******************************************************************************
*/




#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"
#include "ecSysTick.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

unsigned int cnt = 0;
	
void setup(void);
void LED_Toggle(void);
void EXTI15_10_IRQHandler(void);
void Display_Count(void);

int main(void) {
	
	setup();

	while (1);
	
	
}


void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)){
			cnt++;
			sevensegment_display(cnt % 10);
			if(cnt > 9) cnt = 0;
		
		for(int i=0;i<300000;i++);
		
			clear_pending_EXTI(BUTTON_PIN);
		}
	
}


// Initialiization 
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
}

void LED_Toggle(void){
	GPIOA->ODR ^= (1UL << LED_PIN);	 		
}
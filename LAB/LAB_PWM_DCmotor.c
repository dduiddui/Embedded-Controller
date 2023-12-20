/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  Tutorial PWM
*					 - Duwon Yang
* 
******************************************************************************
*/
#include "stm32f411xe.h"
#include "math.h"

// #include "ecSTM32F411.h"
#include "ecPinNames.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecEXTI.h"
#include "ecPWM.h"   // ecPWM2.h


// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN 13
#define PWM_PIN PA_0
#define DIR_PIN PC_2
void setup(void);
void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);


uint32_t count = 0;
unsigned int cnt = 0;
float duty = 0;
float period = 1;
int mode = 0;
static int stop  = 0;
int main(void) {
	// Initialization --------------------------------------------------
	setup();	

	// Infinite Loop ---------------------------------------------------
	while(1){
		if(stop == 0)
			PWM_duty(PWM_PIN, duty/period);
		else if(stop == 1)
			PWM_duty(PWM_PIN, (1/period));  // HIGH -> STOP
	}
}


// Initialiization 
void setup(void) {	
	RCC_PLL_init();
	SysTick_init();
	
	// PWM of 20 msec:  TIM2_CH1 (PA_5 AFmode)
	GPIO_init(GPIOA, LED_PIN, OUTPUT);
	// Initialize GPIOC_13 for Input Button
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);

	// Initialize GPIOC_2 for Output Direction
	GPIO_init(GPIOC, DIR_PIN, OUTPUT);
	GPIO_otype(GPIOC, DIR_PIN, EC_PUSH_PULL);
	// EXTI Initialization ------------------------------------------------------	
	EXTI_init(GPIOC, BUTTON_PIN, FALL,0);
	

	PWM_init(PWM_PIN);	
	PWM_period(PWM_PIN, period);   // 1 msec PWM period
	
	TIM_UI_init(TIM3, 1);         //1msec / Time Interrupt of 500msec
	
}




void TIM3_IRQHandler(void){
	if((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF){ // update interrupt flag
		//Create the code to toggle LED by 
		
		if(count > (99+1) * 20){
			
			if(mode == 0){
				duty = 0.75; mode = 1;
			}
			else if(mode == 1){
				duty = 0.25; mode = 0;
			}
			count = 0;
		}
		count++;
		clear_UIF(TIM3);    		// clear by writing 0
	}
}


void EXTI15_10_IRQHandler(void){
	if (is_pending_EXTI(BUTTON_PIN)){
			stop ^= 1 ;
			clear_pending_EXTI(BUTTON_PIN);
		}
}
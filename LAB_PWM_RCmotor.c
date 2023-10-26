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
#define PWM_PIN PA_1
void setup(void);
void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);


uint32_t count = 0;
unsigned int cnt = 0;
float p = 0, duty = 0;
int dir = 0;

int main(void) {
	// Initialization --------------------------------------------------
	setup();	

	// Infinite Loop ---------------------------------------------------
	while(1){
//		LED_Toggle();		

//		for (int i=0; i<5; i++) {						
//			PWM_duty(PWM_PIN, 0.2*i);			
//			delay_ms(1000);
//			if(i==5) i=0;
//		}

//		
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

	
	// EXTI Initialization ------------------------------------------------------	
	EXTI_init(GPIOC, BUTTON_PIN, FALL,0);
	

	PWM_init(PWM_PIN);	
	GPIO_otype(GPIOA, PWM_PIN, EC_PUSH_PULL); 	//if necessary
	GPIO_pupd(GPIOA, PWM_PIN, EC_PU); 					//if necessary
	GPIO_ospeed(GPIOA, PWM_PIN, EC_HIGH);
	PWM_period(PWM_PIN, 20);   // 20 msec PWM period
	//PWM_duty(PWM_PIN, 1.0);
	
	TIM_UI_init(TIM3, 1);         //1msec / Time Interrupt of 500msec
	
}




void TIM3_IRQHandler(void){
	if((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF){ // update interrupt flag
		//Create the code to toggle LED by 
		
		if(count > (99+1) * 5){			
			LED_Toggle();
			
			if(dir == 0){
				duty = (0.5+(2.0/18.0)*p)/20;
				if((int)p==18){ p=0; dir = 1;}
			}
			else if(dir == 1){
				duty = (2.5 - (2.0/18.0)*p)/20;
				if((int)p==18){ p=0; dir = 0;}
			}
			
			PWM_duty(PWM_PIN, duty);
			
			p++;
			
			
			count = 0;
		}
		count++;
		TIM3->SR &= ~TIM_SR_UIF;    		// clear by writing 0
	}
}


void EXTI15_10_IRQHandler(void){
	if (is_pending_EXTI(BUTTON_PIN)){
			p = 0; dir = 0;
			clear_pending_EXTI(BUTTON_PIN);
		}
	
}
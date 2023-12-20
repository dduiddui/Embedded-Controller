/*
******************************************************************************
* @author  SSSLAB
* @Mod		 2023-11-08 by DuwonYang	
* @brief   Embedded Controller:  LAB - Timer Input Capture 
*					 						- with Ultrasonic Distance Sensor
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "math.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecEXTI.h"
#include "ecUART_simple.h"
#include "ecSysTick.h"
#include "ecStepper.h"

long RPM = 5;

void EXTI15_10_IRQHandler(void);
void setup(void);

int main(void){
	
	setup();
	
	while(1){Stepper_step(2048, 1, HALF);}
}

void setup(void){

	RCC_PLL_init(); 
	SysTick_init();
	
	EXTI_init(GPIOC, BUTTON_PIN, FALL,0);
	GPIO_init(PC_13, INPUT);

	Stepper_init2(PB_10, PB_4, PB_5, PB_3);
	Stepper_setSpeed(RPM);
}

void EXTI15_10_IRQHandler(void) {
	if(is_pending_EXTI(BUTTON_PIN)) {
		Stepper_stop();
		clear_pending_EXTI(BUTTON_PIN);
	}
}
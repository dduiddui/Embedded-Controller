/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : DUWON YANG
Student ID       : 21900427
Created          : 05-03-2021
Modified         : 10-27-2023
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for MID_TEST1
/----------------------------------------------------------------*/


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
#define BUTTON_PIN 13
#define PWM_PIN PA_0
#define DIR_PIN PB_6
#define L 10
#define H 11
void setup(void);
void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
static uint32_t count = 0;
static int vel_type = 0;
static int flag = 0;
static unsigned int direction = 1;
static uint32_t period = 1;
int cnt = 0;


int main(void) {	
	// Initialiization --------------------------------------------------------
	setup();

	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
		//direction can't work version
//		GPIO_write(DIR_PIN, HIGH); // HIGH STATE works but, LOW STATE doesn't work
//		
//		if(vel_type==0){
//			//sevensegment_display(0);
//			PWM_duty(PWM_PIN, 1);
//			sevensegment_decoder(0);
//		}
//		else if(vel_type==1){
//			//sevensegment_display(1);
//			PWM_duty(PWM_PIN, 0.5);
//			sevensegment_decoder(L);
//		}
//		else if(vel_type==2){
//			//sevensegment_display(2);
//			PWM_duty(PWM_PIN, 0);
//			sevensegment_decoder(H);
//		}
		
		//direction work version
		GPIO_write(DIR_PIN, LOW); // HIGH STATE works but, LOW STATE doesn't work
		
		if(vel_type==0){
			sevensegment_decoder(0);
			if(direction == HIGH)
				PWM_duty(PWM_PIN, direction);
		}
		else if(vel_type==1){
			sevensegment_decoder(1);
			PWM_duty(PWM_PIN, 0.5);
		}
		else if(vel_type==2){
			sevensegment_decoder(2);
				PWM_duty(PWM_PIN, !direction);
		}
		
		
		
		}
	}

void setup(void){
	RCC_HSI_init();
	sevensegment_display_init();
	//BUTTON PIN CONFIGURATION
	GPIO_init(PC_13, INPUT);
	GPIO_pupd(PC_13, EC_PD);
	
	//LED PIN CONFIGURATION
	GPIO_init  (LED_PIN, OUTPUT);
	GPIO_otype (LED_PIN, EC_PUSH_PULL);
	GPIO_ospeed(LED_PIN,EC_MEDIUM);
	
	//PWM PIN CONFIGURATION
	PWM_init(PWM_PIN);	
	PWM_period(PWM_PIN, period);   // set PWM period as 1msec 
	
	
	GPIO_init(DIR_PIN, INPUT);
	GPIO_otype(DIR_PIN, EC_PUSH_PULL);
        GPIO_ospeed(DIR_PIN, EC_MEDIUM);
        GPIO_pupd(DIR_PIN, EC_NONE);
	//BUTTON EXTI CONFIGURATION
	EXTI_init(GPIOC, BUTTON_PIN, RISE,0);
	
	//TIMER INTERRUPT CONFIGURATION
	TIM_UI_init(TIM3, 1);         //1msec for Time Interrupt of 500msec
}



void TIM3_IRQHandler(void){
	if((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF){ // update interrupt flag
	
		
		if(count > (99+1) * 5){		
			if(vel_type == 0) LED_OFF();           	 //vel level 0 : LED OFF
			else if(vel_type == 1 && flag == 1){   	 //vel level 1 : LED Blink per 1sec
				LED_Toggle();
				flag = 0;
			}
			else if(vel_type == 1 && flag==0) flag=1;//vel level 1 : LED Blink flag
			else if(vel_type == 2) LED_Toggle();		 //vel level 2 : LED Toggle per 0.5sec

			count = 0;															 //initialize
		}

		count++;
		clear_UIF(TIM3);     		// clear by writing 0
	}
}

void EXTI15_10_IRQHandler(void){
	if (is_pending_EXTI(BUTTON_PIN)){
			vel_type++;
			if(vel_type==3){
				vel_type = 0;
				if(direction == 0) direction = 1;
				else direction = 0;
			}
			
			for(int i = 0; i <300000; i++); 		//Button debouncing
			clear_pending_EXTI(BUTTON_PIN);
		}
}

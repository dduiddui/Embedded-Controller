/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : DUWON YANG
Created          : 05-03-2021
Modified         : 10-13-2023
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_RCC
/----------------------------------------------------------------*/
#include "math.h"
#include "string.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"
#include "ecSysTick.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecPinNames.h"
#include "ecUART.h"
#include "ecStepper.h"
#include "ecADC.h"
#include "stm32f411xe.h"
// #include "ecSTM32F411.h"

#define DIR_LEFT PC_2
#define DIR_RIGHT PC_3
#define PWM_LEFT PA_0
#define PWM_RIGHT PA_1
#define TRIG PA_6
#define ECHO PB_6

void MCU_init(void);
PinName_t seqCHn[2] = {PB_0, PB_1};
float period = 1000;

void MCU_init(void){
RCC_PLL_init();
	SysTick_init();							// SysTick Init
	
	// USB serial unit
	UART2_init();
  UART2_baud(BAUD_9600);
	// BT serial init
  UART1_init();
  UART1_baud(BAUD_9600);
	// PWM1
  PWM_init(PWM_LEFT);
  PWM_period_us(PWM_LEFT, period);
  // PWM2
  PWM_init(PWM_RIGHT);
  PWM_period_us(PWM_RIGHT, period);
	// DIR1 SETUP
  GPIO_init(DIR_LEFT, OUTPUT);
  GPIO_otype(DIR_LEFT, EC_PUSH_PULL);
	GPIO_write(DIR_LEFT, 1);
  // DIR2 SETUP
  GPIO_init(DIR_RIGHT, OUTPUT);
  GPIO_otype(DIR_RIGHT, EC_PUSH_PULL);
	GPIO_write(DIR_RIGHT, 1);
	// LED setup
	GPIO_init(PA_5, OUTPUT);
  // ADC Init
	ADC_init(PB_0);
	ADC_init(PB_1);
	// ADC channel sequence setting
	ADC_sequence(seqCHn, 2);	
	// period 50us (x) -> 1ms (o)

	TIM_UI_init(TIM3, 1);
	// PWM configuration ---------------------------------------------------------------------	
	PWM_init(TRIG);							  	// PA_6: Ultrasonic trig pulse
	PWM_period_us(TRIG, 50000);     // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG, 10);    // PWM pulse width of 10us
	// Input Capture configuration -----------------------------------------------------------------------	
	ICAP_init(ECHO);    							// PB_6 as input caputre
 	ICAP_counter_us(ECHO, 10);   			// ICAP counter step time as 10us
	ICAP_setup(ECHO, IC_1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO, IC_2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect
}
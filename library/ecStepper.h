#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecPinNames.h"

#ifndef __EC_STEPPER_H
#define __EC_STEPPER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//State mode
#define HALF 0
#define FULL 1	 
	 
/* Stepper Motor */
//stepper motor function

typedef struct{
	
	GPIO_TypeDef *port1;
	int pin1;
	GPIO_TypeDef *port2;
	int pin2;
	GPIO_TypeDef *port3;
	int pin3;
	GPIO_TypeDef *port4;
	int pin4;
	uint32_t _step_num;
} Stepper_t;

	 
typedef struct{
	
	PinName_t A;
	PinName_t B;
	PinName_t AN;
	PinName_t BN;
	
	uint32_t _step_num;
	
	
} Stepper2_t;


void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
void Stepper_init2(PinName_t pinA, PinName_t pinB,  PinName_t pinAN, PinName_t pinBN);
void Stepper_setSpeed(long whatSpeed);
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode); 
void Stepper_stop(void);
void configure_stepper(int pins[], int numPins);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

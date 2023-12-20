/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : DUWON YANG
Created          : 05-03-2021
Modified         : 09-30-2023
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/


#include "stm32f411xe.h"
#include "ecRCC.h"
#include "ecPinNames.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

#define HIGH 1
#define LOW  0

#define LED_PIN 	PA_5
#define BUTTON_PIN 13

#define EC_ANG	3
#define EC_ALTE	2
#define EC_DOUT	1
#define EC_DIN	0
#define DIN	0

#define EC_NONE	0
#define EC_PU  	1
#define EC_PD 	2

#define EC_PUSH_PULL  0
#define EC_OPEN_DRAIN	1

#define EC_LOW	  0
#define EC_MEDIUM	1
#define EC_HIGH	  2



#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
void GPIO_init(PinName_t pinName, unsigned int mode); 
void GPIO_write(PinName_t pinName, unsigned int Output);
void GPIO_mode(PinName_t pinName, unsigned int mode);
void GPIO_ospeed(PinName_t pinName, unsigned int speed);
void GPIO_otype(PinName_t pinName, unsigned int type);
void GPIO_pupd(PinName_t pinName, unsigned int pupd);
//void GPIO_init(GPIO_TypeDef *Port, unsigned int pin, unsigned int mode);
//void GPIO_write(GPIO_TypeDef *Port, unsigned int pin, unsigned int Output);
unsigned int GPIO_read(PinName_t pinName);
//void GPIO_mode(GPIO_TypeDef* Port, unsigned int pin, unsigned int mode);
//void GPIO_ospeed(GPIO_TypeDef* Port, unsigned int pin, unsigned int speed);
//void GPIO_otype(GPIO_TypeDef* Port, unsigned int pin, unsigned int type);
//void GPIO_pupd(GPIO_TypeDef* Port, unsigned int pin, unsigned int pupd);

void sevensegment_init(void);
void sevensegment_decoder(unsigned int num);
void sevensegment_display_init(void); 
void sevensegment_display(unsigned int num);
void fourLED_display(unsigned int num);
void LED_Toggle(void);
void LED_OFF(void);
void configure_sevensegment(int pins[], int numPins);
void configure_decoder(int pins[], int numPins);
 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

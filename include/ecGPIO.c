/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : DUWON YANG
Created          : 05-03-2021
Modified         : 09-30-2023
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecPinNames.h"

int seven_pins[] = {PA_5, PA_6, PA_7, PB_6, PC_7, PA_9, PA_8, PB_10};
int seven_numPin = sizeof(seven_pins);
int decoder_pins[] = {PA_7, PB_6, PC_7, PA_9};
int decoder_numPin = sizeof(decoder_pins);

void GPIO_init(PinName_t pinName, unsigned int mode){  
	
	 GPIO_TypeDef *port;
   unsigned int pin;
   ecPinmap(pinName, &port, &pin);
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (port == GPIOA)
		RCC_GPIOA_enable();
	if (port == GPIOB)
		RCC_GPIOB_enable();
	if (port == GPIOC)
		RCC_GPIOC_enable();
	if (port == GPIOD)
		RCC_GPIOD_enable();
	
	if(mode == OUTPUT || mode == AF){
		port->OSPEEDR &= ~(3UL<<(2*pin));
		port->OSPEEDR |=   2UL<<(2*pin);  //speed high
		port->OTYPER  |= EC_PUSH_PULL;    //push-pull
	}
	GPIO_mode(pinName, mode);
	
}


// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(PinName_t pinName, unsigned int mode){
	 GPIO_TypeDef *port;
   unsigned int pin;
   ecPinmap(pinName, &port, &pin);
   port->MODER &= ~(3UL<<(2*pin));     
   port->MODER |= mode<<(int)(2*pin);    
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(PinName_t pinName, unsigned int speed){
	GPIO_TypeDef *port;
  unsigned int pin;
  ecPinmap(pinName, &port, &pin);
	port->OSPEEDR &= ~(3UL<<(2*pin));
	port->OSPEEDR |= speed<<(2*pin);
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(PinName_t pinName, unsigned int type){
	GPIO_TypeDef *port;
  unsigned int pin;
  ecPinmap(pinName, &port, &pin);  
	port->OTYPER &= ~(1UL<<pin);
	port->OTYPER |= type<<(pin);
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(PinName_t pinName, unsigned int pupd){
  GPIO_TypeDef *port;
  unsigned int pin;
  ecPinmap(pinName, &port, &pin); 
	port->PUPDR &= ~(3UL << (2*pin));
	port->PUPDR |= pupd<<(2*pin);
}

unsigned int GPIO_read(PinName_t pinName){
  GPIO_TypeDef *port;
  unsigned int pin;
  ecPinmap(pinName, &port, &pin); 
	unsigned int Val = 0;
	Val = (port->IDR>>pin)&(1UL);
	return Val;    	//[TO-DO] YOUR CODE GOES HERE	
}

void GPIO_write(PinName_t pinName, unsigned int Output){
	GPIO_TypeDef *port;
  unsigned int pin;
  ecPinmap(pinName, &port, &pin); 
	if (Output == 0)
		port->ODR &= ~(1UL << pin);
	else{
		port->ODR |= (Output<<pin);
	}
}

void sevensegment_init(void){
			
		 GPIO_init(PC_13, INPUT);
		 GPIO_pupd(PC_13,EC_PU);
		configure_sevensegment(seven_pins, seven_numPin);
	

}
void sevensegment_decoder(unsigned int num){
	
   unsigned int number[12][8]={
		 
						
						   {1,1,1,1,1,1,0,0}, //zero
						   {0,1,1,0,0,0,0,0}, //one
						   {1,1,0,1,1,0,1,0}, //two
						   {1,1,1,1,0,0,1,0}, //three
						   {0,1,1,0,0,1,1,0}, //four
						   {1,0,1,1,0,1,1,0}, //five
						   {0,0,1,1,1,1,1,0}, //six
						   {1,1,1,0,0,1,0,0}, //seven
						   {1,1,1,1,1,1,1,0}, //eight
						   {1,1,1,1,0,1,1,0}, //nine
							 {0,0,0,1,1,1,0,0}, //L
							 {0,1,1,0,1,1,1,0}, //H
							 
      };
		
		GPIO_write(PA_5, !number[num][4]); //e
		GPIO_write(PA_7, !number[num][2]); //c
		GPIO_write(PB_6, !number[num][7]); //dot
		GPIO_write(PC_7, !number[num][6]); //g
		GPIO_write(PA_9, !number[num][5]); //f
		GPIO_write(PA_8, !number[num][0]); //a
		GPIO_write(PB_10,!number[num][1]); //b
		GPIO_write(PA_6, !number[num][3]); //d
			
}
	
void sevensegment_display_init(void){
		
		GPIO_init(PC_13, INPUT);
		GPIO_pupd(PC_13,EC_PU);

		configure_sevensegment(seven_pins, seven_numPin);
		//configure_decoder(decoder_pins, decoder_numPin);
}
void sevensegment_display(unsigned int num){
	
unsigned int number_display[10][4]={
		 
						
						   {0,0,0,0}, //zero
						   {0,0,0,1}, //one
						   {0,0,1,0}, //two
						   {0,0,1,1}, //three
						   {0,1,0,0}, //four
						   {0,1,0,1}, //five
						   {0,1,1,0}, //six
						   {0,1,1,1}, //seven
						   {1,0,0,0}, //eight
						   {1,0,0,1}, //nine
							 
      };
		
		GPIO_write(PA_7, number_display[num][3]); //a
		GPIO_write(PB_6, number_display[num][2]); //b
		GPIO_write(PC_7, number_display[num][1]); //c
		GPIO_write(PA_9, number_display[num][0]); //d
	
}

void fourLED_display(unsigned int num){
	
unsigned int number_display[16][4]={
		 
						
						   {0,0,0,0}, //zero
						   {0,0,0,1}, //one
						   {0,0,1,0}, //two
						   {0,0,1,1}, //three
						   {0,1,0,0}, //four
						   {0,1,0,1}, //five
						   {0,1,1,0}, //six
						   {0,1,1,1}, //seven
						   {1,0,0,0}, //eight
						   {1,0,0,1}, //nine
							 {1,0,1,0}, //ten
							 {1,0,1,1}, //eleven
							 {1,1,0,0}, //twelve
							 {1,1,0,1}, //thirteen
							 {1,1,1,0}, //fourteen
							 {1,1,1,1}, //fifteen
      };
		
		GPIO_write(PA_0, number_display[num][3]); //a
		GPIO_write(PA_1, number_display[num][2]); //b
		GPIO_write(PB_0, number_display[num][1]); //c
		GPIO_write(PC_1, number_display[num][0]); //d
	
}

void LED_Toggle(void){

				GPIOA->ODR ^= (1UL << LED_PIN);	 		
}

void LED_OFF(void){
	GPIOA->ODR &= ~(1UL << LED_PIN);
}

void configure_sevensegment(int pins[], int numPins) {
    for (int i = 0; i < numPins; i++) {
        GPIO_init(pins[i], OUTPUT);
        GPIO_ospeed(pins[i], EC_MEDIUM);
        GPIO_otype(pins[i], EC_PUSH_PULL);
        GPIO_pupd(pins[i], EC_NONE);
    }
}

void configure_decoder(int pins[], int numPins) {
	for (int i = 0; i < numPins; i++) {
        GPIO_init(pins[i], OUTPUT);
        GPIO_ospeed(pins[i], EC_MEDIUM);
        GPIO_otype(pins[i], EC_PUSH_PULL);
        GPIO_pupd(pins[i], EC_NONE);
    }
}
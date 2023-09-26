/**
******************************************************************************
* @author	Duwon Yang
* @Mod		2023.09.24
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle LED LD2 by Button B1 pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization
	setup();
	
	// Inifinite Loop 
	while(1){
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0)		GPIO_write(GPIOA, LED_PIN, LOW);
		else 										GPIO_write(GPIOA, LED_PIN, HIGH);
	}
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_mode(GPIOA, LED_PIN, OUTPUT);    		// Set GPIOC as OUTPUT
	GPIO_pupd(GPIOA, LED_PIN, EC_PU);     		// Set GPIOC as PULL_UP
	GPIO_otype(GPIOA, LED_PIN, EC_OPEN_DRAIN); // Set GPIOC as PUSH_PULL
	GPIO_ospeed(GPIOA, LED_PIN, EC_MEDIUM);   // Set GPIOC as MEDIUM SPEED
}

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

#define LED_PIN_1 	5
#define LED_PIN_2 	6
#define LED_PIN_3 	7
#define LED_PIN_4 	6
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();	
	int rep = 1;
	int delay = 0;
	
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		


		if(GPIO_read(GPIOC, BUTTON_PIN) == 0) {
			if((rep == 1)&&(delay > 100000)){
				GPIO_write(GPIOA, LED_PIN_1, HIGH);
				GPIO_write(GPIOB, LED_PIN_4, LOW);
				rep++;
				delay = 0;
			}
			else if((rep == 2)&&(delay > 100000)){
				GPIO_write(GPIOA, LED_PIN_1, LOW);
				GPIO_write(GPIOA, LED_PIN_2, HIGH);
				rep++;
				delay = 0;
			}
			else if((rep == 3)&&(delay > 100000)){
				GPIO_write(GPIOA, LED_PIN_2, LOW);
				GPIO_write(GPIOA, LED_PIN_3, HIGH); 
				rep++;
				delay = 0;
			}
			else if((rep == 4)&&(delay > 100000)){
				GPIO_write(GPIOA, LED_PIN_3, LOW); 
				GPIO_write(GPIOB, LED_PIN_4, HIGH);
				rep = 1;
				delay = 0;
			}
		}
				delay++;
	}
}


// Initialiization 
void setup(void)
{

	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  		// calls RCC_GPIOC_enable()
	GPIO_mode(GPIOC, BUTTON_PIN, INPUT);  		// Set GPIOC as INPUT
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);  		// Set GPIOC as PULL_UP
	
	
	GPIO_init(GPIOA, LED_PIN_1, OUTPUT);    		// calls RCC_GPIOA_enable()
	GPIO_mode(GPIOA, LED_PIN_1, OUTPUT);    		// Set GPIOC as OUTPUT
	GPIO_pupd(GPIOA, LED_PIN_1, EC_PU);     		// Set GPIOC as PULL_UP
	GPIO_otype(GPIOA, LED_PIN_1, EC_PUSH_PULL); // Set GPIOC as PUSH_PULL
	GPIO_ospeed(GPIOA, LED_PIN_1, EC_MEDIUM);   // Set GPIOC as MEDIUM SPEED
	
	GPIO_init(GPIOA, LED_PIN_2, OUTPUT);    		// calls RCC_GPIOA_enable()
	GPIO_mode(GPIOA, LED_PIN_2, OUTPUT);    		// Set GPIOC as OUTPUT
	GPIO_pupd(GPIOA, LED_PIN_2, EC_PU);     		// Set GPIOC as PULL_UP
	GPIO_otype(GPIOA, LED_PIN_2, EC_PUSH_PULL); // Set GPIOC as PUSH_PULL
	GPIO_ospeed(GPIOA, LED_PIN_2, EC_MEDIUM);   // Set GPIOC as MEDIUM SPEED
	
	GPIO_init(GPIOA, LED_PIN_3, OUTPUT);    		// calls RCC_GPIOA_enable()
	GPIO_mode(GPIOA, LED_PIN_3, OUTPUT);    		// Set GPIOC as OUTPUT
	GPIO_pupd(GPIOA, LED_PIN_3, EC_PU);     		// Set GPIOC as PULL_UP
	GPIO_otype(GPIOA, LED_PIN_3, EC_PUSH_PULL); // Set GPIOC as PUSH_PULL
	GPIO_ospeed(GPIOA, LED_PIN_3, EC_MEDIUM);   // Set GPIOC as MEDIUM SPEED
	
	GPIO_init(GPIOB, LED_PIN_4, OUTPUT);    		// calls RCC_GPIOA_enable()
	GPIO_mode(GPIOB, LED_PIN_4, OUTPUT);    		// Set GPIOC as OUTPUT
	GPIO_pupd(GPIOB, LED_PIN_4, EC_PU);     		// Set GPIOC as PULL_UP
	GPIO_otype(GPIOB, LED_PIN_4, EC_PUSH_PULL); // Set GPIOC as PUSH_PULL
	GPIO_ospeed(GPIOB, LED_PIN_4, EC_MEDIUM);   // Set GPIOC as MEDIUM SPEED
	

}
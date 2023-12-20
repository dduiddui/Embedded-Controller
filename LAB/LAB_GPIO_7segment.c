#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

void setup(void);
	
int main(void) {	
	// Initialiization --------------------------------------------------------
	setup();
	unsigned int cnt = 0;
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		sevensegment_display(cnt % 10);
		if(GPIO_read(GPIOC, BUTTON_PIN)==0) cnt++;
		if(cnt > 9) cnt = 0;
		for(int i=0;i<300000;i++){}
	}
}

void setup(void){
	RCC_HSI_init();
	sevensegment_display_init();
}
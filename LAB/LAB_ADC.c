#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecSysTick.h"
#include "ecUART.h"
#include "ecADC.h"
#include "ecPinNames.h"



void setup(void);

int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		printf("value1 = %d \r\n",value1);
		printf("value2 = %d \r\n",value2);
		printf("\r\n");
		delay_ms(1000);
	}
}

// Initialiization 
void setup(void)
{	
	RCC_PLL_init();                         // System Clock = 84MHz
	UART2_init();							// UART2 Init
	SysTick_init();							// SysTick Init
	
	// ADC Init
	ADC_init(PB_0);
	ADC_init(PB_1);

	// ADC channel sequence setting
	ADC_sequence(seqCHn, 2);
}


void ADC_IRQHandler(void){
	if(is_ADC_OVR())
		clear_ADC_OVR();
	
	if(is_ADC_EOC()){		// after finishing sequence
		if (flag==0)
			value1 = ADC_read();  
		else if (flag==1)
			value2 = ADC_read();
			
		flag =! flag;		// flag toggle
	}
}

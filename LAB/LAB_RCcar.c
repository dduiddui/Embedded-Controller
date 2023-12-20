/*
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  LAB Bluetooth
*					 - Duwon Yang
* 
******************************************************************************
*/

#include "ecSTM32F411.h"


// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN 13
#define PWM_PIN PA_0
#define DIR_LEFT PC_2
#define DIR_RIGHT PC_3
#define PWM_LEFT PA_0
#define PWM_RIGHT PA_1
#define STOP 1
#define UP 72
#define DOWN 80
#define TRIG PA_6
#define ECHO PB_6
void setup(void);
void USART1_IRQHandler(void);
void ADC_IRQHandler(void);
void Auto_mode (void);
	
static volatile unsigned char MODE;
static volatile unsigned char DIRECTION;
static volatile unsigned char STR1;
static volatile unsigned char STR2;
static volatile unsigned char VEL;

static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;

//IR parameter//
uint32_t value1, value2;
int flag = 0;
int vel = -1;
int str= 0;
int DIR = 1;
int mode= 0;
int drive_state = 1;
int straight = 1;
int cnt = 0;
static volatile float right_duty = 1, left_duty = 1;

uint32_t ovf_cnt = 0;
float distance = 0;
float last_distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;
int TIM3count = 0;

float dist[10] = {0.0};
int dist_index = 0;
float dist_sum = 0;


void USART1_IRQHandler(void);
void manipulate(void);
void manual_mode (volatile uint8_t BT_Data)	;
void Auto_mode (void)		;
void USART_print (void);



int main(void) { 
	// Initialization --------------------------------------------------
	  setup();

	while(1){

		if(mode==1) {manual_mode(BT_Data);
			USART1_write((uint8_t *)"Start",5);
		}
		else if(mode == 2) Auto_mode();
		
		PWM_duty(PWM_LEFT,   left_duty);
		PWM_duty(PWM_RIGHT,  right_duty);
	delay_ms(500);
	}
	
}

// Initialiization 
void setup(void) {	
	MCU_init();
}


void TIM3_IRQHandler(void) {
    if (is_UIF(TIM3)) { //10Hz
        TIM3count++;
        if (TIM3count >= 20) {
            if (mode == 2) LED_Toggle();
            else GPIO_write(LED_PIN, HIGH);
						USART_print();
            TIM3count = 0;
        }
    }
    clear_UIF(TIM3);
}



void USART1_IRQHandler(){                       // USART2 RX Interrupt : Recommended
    if(is_USART1_RXNE()){
        BT_Data = USART1_read();                // RX from UART1 (BT)
			
				USART_write(USART1,(uint8_t*) "BT sent : ", 10);
				USART_write(USART1,&BT_Data , 1);
				USART_write(USART1, "\r\n", 2);
			
			if(BT_Data == 'M' || BT_Data == 'm') 
			{	
				//MODE CHANGE & DIRECTION SETTING
				mode = 1;
				GPIO_write(PC_2, 1);
				GPIO_write(PC_3, 1);
			}
			
			if(BT_Data == 'Q' || BT_Data == 'q')
			{
				mode = 2;
				GPIO_write(PC_2, 1);
				GPIO_write(PC_3, 1);
			}
			
			if(BT_Data == 'H' || BT_Data == 'h')
			{
				if(drive_state==1 && vel!=3) vel++;
				else if(drive_state==2 && str!=3) str++;
				else if(drive_state==3 && str!=3) str++;
			}
			if(BT_Data == 'N' || BT_Data == 'n')
			{
				if(drive_state==1 && vel!=0) vel--;
				else if(drive_state==2 && str!=-3) str--;
				else if(drive_state==3 && str!=-3) str--;
			}		
			if(BT_Data == 'A' || BT_Data == 'a')
				drive_state = 3;
			if(BT_Data == 'D' || BT_Data == 'd')
				drive_state = 2;
			if(BT_Data == 'S' || BT_Data == 's')
				vel = -1;
			if(BT_Data == 'W' || BT_Data == 'w' || BT_Data == 'B' || BT_Data == 'b')
				vel = 0;
	}
}




// move as direction
void manipulate(void){
	
	switch(drive_state){
		case 0:  //STOP
			left_duty  = fabs(DIR-1);
			right_duty = fabs(DIR-1);
			break;
		case 1:  // GO STRAIGHT
			left_duty  = fabs(DIR-(0.57+0.1*vel));
			right_duty = fabs(DIR-(0.57+0.1*vel));
			break;
		case 2:  // TURN RIGHT
			str = 1;
			left_duty   = fabs(DIR-(0.7));
			right_duty  = fabs(DIR-(0.7+(str*0.1)));
			
			break;
		case 3:	 // TURN LEFT
			str = -1;
			left_duty   = fabs(DIR-(0.7));
			right_duty  = fabs(DIR-(0.7+(str*0.1)));
			
			break;
		}													
}																

void manual_mode (volatile uint8_t BT_Data){		
				
	GPIO_write(LED_PIN, 1);
				switch(BT_Data){
		
					case 'H':
					case 'h':
					case 'N':
					case 'n':					
					case 'D':
					case 'd':					
					case 'A':
					case 'a':
						manipulate();
						break;
					//STOP
					case 'S':
					case 's':
						drive_state = 0;
						if(straight == 1) DIR = 0;
						else DIR = 1;
						manipulate();
						break;
					//GO STRAIGHT
					case 'W':
					case 'w':
						drive_state = 1;
						straight = 1;
						DIR = 1;
						GPIO_write(DIR_LEFT, DIR);
						GPIO_write(DIR_RIGHT, DIR);
						
						manipulate();
						break;
		
					//GO BACKWARD
					case 'B':
					case 'b':
						drive_state = 1;
						straight = 0;
						DIR=0;	
						GPIO_write(DIR_LEFT, DIR);
						GPIO_write(DIR_RIGHT, DIR);
						manipulate();
						break;
			
			}

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




void TIM4_IRQHandler(void){
	if(is_UIF(TIM4)){                     			// Update interrupt 
		ovf_cnt++			 ;													// overflow count
		clear_UIF(TIM4);  							   				// clear update interrupt flag
	}
	if(is_CCIF(TIM4, IC_1)){ 										// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1 = ICAP_capture(TIM4, IC_1);					// Capture TimeStart
		clear_CCIF(TIM4, 1);                			// clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM4, IC_2)){ 							// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		time2 = ICAP_capture(TIM4, IC_2);					// Capture TimeEnd
		timeInterval = ((time2 - time1) + ovf_cnt * (TIM2->ARR + 1)) / 100; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		ovf_cnt = 0;                        			// overflow reset
		clear_CCIF(TIM4,2);								  			// clear capture/compare interrupt flag 
	}
}


void Auto_mode (void){
				
				dist[dist_index] = distance;
				for(int i=0; i<10; i++){
					if (dist[i] < 10 && dist[i] >0) cnt++; 
				}
				dist_index ++;
				dist_index %= 10;		
				
				if( cnt > 7 ){
						left_duty = 1;
					  right_duty = 1;
						vel = -1;
						str = 0;
				}
				else{
					if(value1 > 1500 && value2 > 1500){
						left_duty  = 1;
						right_duty = 1;
						vel = -1;
						str = 0;
					}	
					else if(value2 > 1500){
						left_duty = 1-1;
						right_duty = 1-0.5;
						str = 1;
						vel = 0;
					}
					else if(value1 > 1500){
						left_duty = 1-0.5;
						right_duty = 1-1;
						str = -1;
						vel = 0;
					}
					else {
						left_duty = 1-1;
						right_duty = 1-1;
						str = 0;
						vel = 0;
					}
				}
				last_distance = distance; 
				
				cnt = 0;
				delay_ms(10);
}


void USART_print (void){
   
	
	 if(mode == 1 || mode == 2)
			USART1_write((uint8_t *)"MOD: ",5);
   
	 switch(mode){
      case 1:
         USART1_write((uint8_t *)"M",1);
         break;
      case 2:
         USART1_write((uint8_t *)"A",1);
         break;
   }
	 
   if(mode == 1){
			 USART1_write((uint8_t *)" DIR: ",5);
			 switch(DIR){
					case 0:
						 USART1_write((uint8_t *)"0",1);
						 break;
					case 1:
						 USART1_write((uint8_t *)"1",1);
						 break;
			 }
			 
			 USART1_write((uint8_t *)" STR: ",5);
					 switch(str){
						 case -3:
								 USART1_write((uint8_t *)"-3",2);
						 break;
						 case -2:
								 USART1_write((uint8_t *)"-2",2);
						 break;
						 case -1:
								 USART1_write((uint8_t *)"-1",2);
						 break;
					case 0:
						 USART1_write((uint8_t *)"0",2);
						 break;
					case 1:
						 USART1_write((uint8_t *)"1",2);
						 break;
						 case 2:
								 USART1_write((uint8_t *)"2",2);
						 break;
						 case 3:
								 USART1_write((uint8_t *)"3",2);
						 break;
			 }
				
					
			 USART1_write((uint8_t *)" VEL: ",5);
				
				switch(vel){
						 case -1:
								 USART1_write((uint8_t *)"STOP",4);
								 break;
						 case 0:
								 USART1_write((uint8_t *)"V0",2);
								 break;
						 case 1:
								 USART1_write((uint8_t *)"V1",2);
								 break;
						 case 2:
								 USART1_write((uint8_t *)"V2",2);
								 break;
						 case 3:
								 USART1_write((uint8_t *)"V3",2);
								 break;
			 }
				
			 USART1_write((uint8_t *)"\r\n",2);
	 }
	 
	 
	 else if(mode==2){
		 USART1_write((uint8_t *)" DIR: ",5);
		 USART1_write((uint8_t *)"1",1);
		 USART1_write((uint8_t *)" STR: ",5);
		 switch(str){
					case -1:
								 USART1_write((uint8_t *)"-1",2);
						 break;
					case 0:
						 USART1_write((uint8_t *)"0",2);
						 break;
					case 1:
						 USART1_write((uint8_t *)"1",2);
						 break;
			}		 
		 USART1_write((uint8_t *)" VEL: ",5);
		 switch(vel){
						 case -1:
								 USART1_write((uint8_t *)"STOP",4);
								 break;
						 case 0:
								 USART1_write((uint8_t *)"V0",2);
								 break;
			 }
		 USART1_write((uint8_t *)"\r\n",2);
		 
	 }
   
   
}


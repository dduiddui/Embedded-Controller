/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HAEUN KIM
Created          : 05-03-2021
Modified         : 11-17-2023
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_RC_Car
/----------------------------------------------------------------*/

#include "ecSTM32F411.h"
#define END_CHAR  13
#define MAX_BUF  100

// Motor for elevator
#define PWM_PIN PA_0
#define DIR_PIN 2

// Motor for fan
#define FAN_PWM_PIN PA_1

// LED for Mode
#define RED_LED    8
#define GRN_LED    6
#define PLT_LED		10 // PB10
#define YLW_LED    5 // PA5

// Bluetooth

// Temp and Humi
#define TEMP_PIN 7 // GPIOC
#define HUMI_PIN 6 // GPIOB

// Servo motor for window
#define SERVO_PWM_PIN PB_4


static volatile uint32_t pwm_period = 1;
static volatile float 	 duty = 0;
static volatile float 	 FAN_duty = 1.0;
static volatile uint32_t Button = 0;
static volatile uint32_t Button_state = 0;
static volatile uint32_t dir = 0;
static volatile uint8_t	 Mode = 0;
static volatile uint8_t	 cmd = 0;
static volatile uint32_t servo_pwm_period = 20; // 20 [msec]

//-------------------USART1 Parameter-----------------//
static volatile uint8_t bReceived = 0;
static volatile uint8_t BT_Data   = 0;

static volatile uint8_t buffer[MAX_BUF]    = {0, };
static volatile uint8_t BT_string[MAX_BUF] = {0, };

static volatile uint8_t  idx = 0;

static volatile char s1[40];

//-------------------USART2 Parameter-----------------//
static volatile uint8_t cReceived = 0;
static volatile uint8_t CT_Data   = 0;

static volatile uint8_t buf[MAX_BUF]    = {0, };
static volatile uint8_t CT_string[MAX_BUF] = {0, };

static volatile uint8_t  i = 0;


//-------------Timer Interrupt parameter------------//
static volatile uint16_t A_cnt = 0;
static volatile uint16_t M_cnt = 0;
static volatile uint16_t count = 0;

//-------------------Light parameter----------------//
static volatile uint32_t Light_value  = 0;
static volatile uint32_t value1 = 0;
static volatile uint32_t value2 = 0;
static volatile uint32_t flag   = 0;
static volatile int sequence[2] = {8,9};


//---------Temperature and Humidity parameter-------//
static volatile unsigned int Temp_value = 0;
static volatile unsigned int Humi_value = 1;



//---------------------Functions--------------------//

void MCU_init(void);
void MOTOR_init(void);
void setup(void);
void EXTI15_10_IRQHandler(void);
void Button_pressed(void);
void Operating(void);
void Motor_Stop(PinName_t PWM_pinName, int DIR_pin);
void Mode_select(uint8_t MOD);
void USART1_IRQHandler(void);
void Manual_cmd(uint8_t command);
void Automatic_mode(void);
void Manual_mode(void);
void TIM5_IRQHandler(void);
void Light_init(void);
void ADC_IRQHandler(void);
void USART2_IRQHandler(void);
void TempHumid_init(void);
void Button_LED(void);
void Humid_check(void);

int main(void)
{
	// Initialiization --------------------------------------------------------
	setup();
	USART1_write("Press Button Once : Service Version\r\n", 37);
	USART1_write("Press Button Twice : Operating Version\r\n",40);
	GPIO_write(GPIOB, PLT_LED, HIGH);

	// Inifinite Loop ----------------------------------------------------------
	while(1){
		//GPIO_write(GPIOC, DIR_PIN, LOW); // (1, 0.5) (0, 0.25)
		//PWM_duty(PWM_PIN, duty);

	}
}


//==================================Set-Up==================================//
// Initialiization 
void setup(void)
{	
	MCU_init();          
}

void MCU_init(void){
	RCC_PLL_init();
	SysTick_init(1);        // priority = 8
	
	UART2_init();
	UART2_baud(BAUD_9600);
	
	UART1_init();
	UART1_baud(BAUD_9600);
	
	// Plant LED
	GPIO_init(GPIOB, PLT_LED, OUTPUT);
	GPIO_pupd(GPIOB, PLT_LED, EC_PU);
	GPIO_ospeed(GPIOB, PLT_LED, EC_MEDIUM);
	
	// INPUT : Button pin Initialization
  GPIO_init(GPIOC, BUTTON_PIN, INPUT);
  GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
  EXTI_init(GPIOC, BUTTON_PIN, FALL, 9);  // priority = 9
	
  // Red LED 
	GPIO_init(GPIOA, RED_LED, OUTPUT);
	GPIO_pupd(GPIOA, RED_LED, EC_PU);
	GPIO_ospeed(GPIOA, RED_LED, EC_MEDIUM);
	
	// Green LED 
	GPIO_init(GPIOA, GRN_LED, OUTPUT);
	GPIO_pupd(GPIOA, GRN_LED, EC_PU);
	GPIO_ospeed(GPIOA, GRN_LED, EC_MEDIUM);
	
	// Yellow LED 
	GPIO_init(GPIOA, YLW_LED, OUTPUT);
	GPIO_pupd(GPIOA, YLW_LED, EC_PU);
	GPIO_ospeed(GPIOA, YLW_LED, EC_MEDIUM);
	
	MOTOR_init();
	
	Light_init();
	
	TIM_UI_init(TIM5, 5); 
	
}

void MOTOR_init(void){
	// OUTPUT : Direction pin Initialization
  GPIO_init(GPIOC, DIR_PIN, OUTPUT);
  GPIO_otype(GPIOC, DIR_PIN, EC_PUSH_PULL);
  GPIO_write(GPIOC, DIR_PIN, LOW);             // DIR = 0 : DOWN / DIR = 1 : UP
	
	// PWM for elevator motor
	PWM_init(PWM_PIN);	
	PWM_period(PWM_PIN, pwm_period);   // 1 msec PWM period
	
	// Direction for fan <- Connected to the 5[V]
	
	// PWM for fan motor
	PWM_init(FAN_PWM_PIN);
	PWM_period(FAN_PWM_PIN, pwm_period);
	
	// PWM for Servo motor
	PWM_init(SERVO_PWM_PIN);	
	PWM_period(SERVO_PWM_PIN, servo_pwm_period); 
	
	duty = 0.0;
	FAN_duty = 1.0;
	PWM_duty(PWM_PIN, duty);
	PWM_duty(FAN_PWM_PIN, FAN_duty);
	PWM_duty(SERVO_PWM_PIN,(float)0.025);
}

void Light_init(void){
	ADC_init(PB_0);
	ADC_init(PB_1);
	
	ADC_sequence(sequence, 2);
}

void TempHumid_init(void){
	GPIO_init(GPIOC, TEMP_PIN, INPUT);
	GPIO_init(GPIOB, HUMI_PIN, INPUT);
	
}


//==================================Functions==================================//
//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {
	if ((EXTI->PR & EXTI_PR_PR13) == EXTI_PR_PR13) {
		delay_ms(500);
		Button++; 
		Button_pressed();
		Button_LED();
		clear_pending_EXTI(BUTTON_PIN); 
	}
}


// Service Ver. or Mode Ver.
void Button_pressed(void){
	Button_state = Button%2 ;
	
	if (Button_state == 1){  // Service Version
		USART1_write("Service Mode.\r\n",15);
		GPIO_write(GPIOC, DIR_PIN, Button_state);   // Direction : UP
		PWM_duty(PWM_PIN, (float)0.2); 
		delay_ms(2000); 
		Motor_Stop(PWM_PIN, DIR_PIN);
		
	} else if (Button_state == 0){  // Mode Version
		PWM_duty(PWM_PIN, (float)0.8); // Direction : DOWN
		delay_ms(2000);
		Motor_Stop(PWM_PIN, DIR_PIN);
		USART1_write("Auto or Manual. (A or M)\r\n",26);
	}
}

void Button_LED(void){
	if (Button_state == 1){  // Service Version
		GPIO_write(GPIOA, RED_LED, LOW);   // LED OFF
		GPIO_write(GPIOA, GRN_LED, LOW);
		GPIO_write(GPIOA, YLW_LED, HIGH);
		GPIO_write(GPIOB, PLT_LED, LOW);
		
	}else if (Button_state == 0){  // Mode Version
		GPIO_write(GPIOA, RED_LED, HIGH);   // LED ON
		GPIO_write(GPIOA, GRN_LED, HIGH);		// LED OFF
		GPIO_write(GPIOA, YLW_LED, LOW);
		GPIO_write(GPIOB, PLT_LED, HIGH);
	}
}

// Stop the Motor 
void Motor_Stop(PinName_t PWM_pinName, int DIR_pin){
	GPIO_TypeDef *port;
	unsigned int pin;	
	ecPinmap(PWM_pinName, &port, &pin);	
	
	GPIO_write(GPIOC, DIR_pin, LOW); 
	PWM_duty(PWM_pinName, (float) 0.0); 
}


// Select the Mode 
void Mode_select(uint8_t MOD){
	if ((MOD == 'M') ||(MOD == 'm')){     
		USART1_write("Manual Mode\r\n", 13);
		Mode = 'M';
	}else if((MOD == 'A') ||(MOD == 'a')){
		USART1_write("Automatic Mode\r\n", 16);
		Mode = 'A';
	}
}
	
void Manual_cmd(uint8_t command){
	switch(command){
		case 'U' :  // Elevator Go Up
			
			{
				USART1_write("Take from the water.\r\n",22);
				dir = 1;
				duty = (float) 0.2; 
				GPIO_write(GPIOC, DIR_PIN, dir);   // Direction : UP
				PWM_duty(PWM_PIN, duty);
				delay_ms(1300);
				Motor_Stop(PWM_PIN, DIR_PIN);
				
				break;  
			} 
		case 'D' :	// Elevator Go Down
			{
				USART1_write("Put under water.\r\n", 18);
				dir = 0;
				duty = (float) 0.8;
				GPIO_write(GPIOC, DIR_PIN, dir);   // Direction : DOWN
				PWM_duty(PWM_PIN, duty);
				delay_ms(1300);
				Motor_Stop(PWM_PIN, DIR_PIN);
				
				break;
			}
		case 'R' :  // Turn Off the Plant LED
			{
				USART1_write("Plant LED OFF.\r\n",16);
				GPIO_write(GPIOB, PLT_LED, LOW) ; 
				
				break;
			}
		case 'L' :  // Turn On the Plant LED
			{
				USART1_write("Plant LED ON.\r\n",15);
				GPIO_write(GPIOB, PLT_LED, HIGH);
				
				break;
			}
		case 'F' :  // Operate the Fan
			{
				USART1_write("Fan ON.\r\n",9);
				PWM_duty(FAN_PWM_PIN, (float)0.0);
				break;
			}
		case 'N' :  // Pause the Fan
			{
				USART1_write("Fan OFF.\r\n",10);
				PWM_duty(FAN_PWM_PIN, (float)0.8);
				break;
			}
		case 'O' :
			{
				USART1_write("Ceiling Open.\r\n",15);
				PWM_duty(SERVO_PWM_PIN, (float)0.95);
				break;
			}
		case 'C' :
			{
				USART1_write("Ceiling Closed.\r\n",17);
				PWM_duty(SERVO_PWM_PIN, (float)0.0);
				break;
			}
	}
}

void Manual_mode(void){
	GPIO_write(GPIOA, RED_LED, HIGH);   // LED ON
	GPIO_write(GPIOA, GRN_LED, LOW);		// LED OFF
	GPIO_write(GPIOA, YLW_LED, LOW); 
}

void Automatic_mode(void){
	GPIO_write(GPIOA, RED_LED, LOW);    // LED OFF
	GPIO_write(GPIOA, GRN_LED, HIGH);		// LED ON
	GPIO_write(GPIOA, YLW_LED, LOW); 
	
	// Light Sensor
	if (Light_value > 1000){
		USART1_write("The sun sets.\r\n",15);
		GPIO_write(GPIOB, PLT_LED, HIGH);  // Turn on the Plant Light
		PWM_duty(SERVO_PWM_PIN,(float)0.0);
	}else{
		USART1_write("The sun rises.\r\n",16);
		PWM_duty(FAN_PWM_PIN, (float)0.0);
		GPIO_write(GPIOB, PLT_LED, LOW);   // Turn off the Plant Light
		PWM_duty(SERVO_PWM_PIN,(float)0.95);
	}
	
	
	// Temperature
	if (Temp_value == 1){    // higher than 20 degree --> fan
		PWM_duty(FAN_PWM_PIN, (float)0.0);
		USART1_write("Turn on the Fan.\r\n",18);
	}else{
		PWM_duty(FAN_PWM_PIN, (float)0.8);
	}
	// Humidity
}


void Humid_check(void){
	if (Humi_value == 0){    // Humidity is lower than 50 %  --> water (elevator DC motor)

		USART1_write("Humidity is lower than 50% : Give water\r\n",41);
		// DOWN
		dir = 0;
		duty = (float) 0.8;
		GPIO_write(GPIOC, DIR_PIN, dir);   // Direction : DOWN
		PWM_duty(PWM_PIN, duty);
		delay_ms(1000);
		Motor_Stop(PWM_PIN, DIR_PIN);
		delay_ms(1000);
		// UP
		dir = 1;
		duty = (float) 0.2; 
		GPIO_write(GPIOC, DIR_PIN, dir);   // Direction : UP
		PWM_duty(PWM_PIN, duty);
		delay_ms(1000);
		Motor_Stop(PWM_PIN, DIR_PIN);
		
	}else if (Humi_value == 1){   // Humidity is higher than 50% --> open window (Servo)
		
		Motor_Stop(PWM_PIN, DIR_PIN);		
	}
}


//==============================TIM5_IRQHandler==============================//
void TIM5_IRQHandler(void){
	if(is_UIF(TIM5)){			// Check UIF(update interrupt flag)
		
		// Manual Mode
		if (Mode == 'M'){
			M_cnt++;
			if (M_cnt > 1000){
				Manual_mode();
				M_cnt = 0;
			}
		}
		
		// Automation Mode
		if(Mode == 'A'){
			A_cnt++;
			if (A_cnt > 1000){
				Automatic_mode();
				Humid_check();
				A_cnt = 0;
			}		
		}
		
		count++;
		if (count > 1000){
			Light_value = (value1 + value2) / 2 ;
			Temp_value = GPIO_read(GPIOC, TEMP_PIN);   // 1 or 0
			Humi_value = GPIO_read(GPIOB, HUMI_PIN);   // 1 or 0
			count = 0;
		}
	}
	clear_UIF(TIM5); 		// Clear UI flag by writing 0
}

//==============================USART1_IRQHandler==============================//

void USART1_IRQHandler(void){          		// USART2 RX Interrupt : Recommended
	if(is_USART1_RXNE()){
		BT_Data = USART1_read();		// RX from UART2 (PC)
		USART_write(USART1, &BT_Data, 1);
		
		
		// Creates a String from serial character receive				
		if(BT_Data != END_CHAR && (idx < MAX_BUF)){	
			buffer[idx] = BT_Data;
			idx++;
			
		}else if (BT_Data== END_CHAR) {												// if(Enter)
			USART_write(USART1, "\r\n", 2);
			bReceived = 1;
			memset(BT_string, 0, sizeof(char) * MAX_BUF);       // reset PC_string;	
			memcpy(BT_string, buffer, sizeof(char) * idx);     	// copy to PC_string;
			memset(buffer, 0, sizeof(char) * MAX_BUF);	        // reset buffer
			idx = 0;
			cmd = BT_string[0];
			
			Mode_select(cmd);
			Manual_cmd(cmd);
			
		}else{																								//  if(idx >= MAX_BUF)			
			idx = 0;							
			memset(BT_string, 0, sizeof(char) * MAX_BUF);       // reset PC_string;
			memset(buffer, 0, sizeof(char) * MAX_BUF);					// reset buffer
		}
	}
}

//==============================USART1_IRQHandler==============================//

void USART2_IRQHandler(void){          		// USART2 RX Interrupt : Recommended
	if(is_USART2_RXNE()){
		CT_Data = USART2_read();		// RX from UART2 (PC)
		USART_write(USART2, &CT_Data, 1);
		
		
		// Creates a String from serial character receive				
		if(CT_Data != END_CHAR && (i < MAX_BUF)){	
			buf[i] = CT_Data;
			i++;
			
		}else if (CT_Data== END_CHAR) {												// if(Enter)
			USART_write(USART2, "\r\n", 2);
			cReceived = 1;
			memset(CT_string, 0, sizeof(char) * MAX_BUF);       // reset PC_string;	
			memcpy(CT_string, buf, sizeof(char) * idx);     	// copy to PC_string;
			memset(buf, 0, sizeof(char) * MAX_BUF);	        // reset buffer
			i = 0;

			
		}else{																								//  if(idx >= MAX_BUF)			
			idx = 0;							
			memset(CT_string, 0, sizeof(char) * MAX_BUF);       // reset PC_string;
			memset(buf, 0, sizeof(char) * MAX_BUF);					// reset buffer
		}
	}
}

//=============================Light Sensor============================//

void ADC_IRQHandler(void){
	if(is_ADC_OVR())
		clear_ADC_OVR();
		
	if(is_ADC_EOC()){		// after finishing sequence
		if (flag==0)
			value1 = ADC_read();  
		else if (flag==1)
			value2 = ADC_read();
		flag = !flag;		// flag toggle
		
	}
}

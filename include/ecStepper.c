#include "stm32f4xx.h"
#include "ecStepper.h"

//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7


// Stepper Motor function
uint32_t direction = 1; 
uint32_t step_delay = 100; 
uint32_t step_per_rev = 64*32;
	 

// Stepper Motor variable
volatile Stepper_t myStepper; 
volatile Stepper2_t myStepper2;

//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out;
  	uint32_t next[2];
} State_full_t;

State_full_t FSM_full[4] = {  
 	{0b1100,{S1,S3}},
 	{0b0110,{S2,S0}},
	{0b0011,{S3,S1}},
	{0b1001,{S0,S2}},
};

//HALF stepping sequence
typedef struct {
	uint8_t out;
  	uint32_t next[2];
} State_half_t;

State_half_t FSM_half[8] = { 
 	{0b1000,{S1,S7}},
	{0b1100,{S2,S0}},
	{0b0100,{S3,S1}},
	{0b0110,{S4,S2}},
	{0b0010,{S5,S3}},
	{0b0011,{S6,S4}},
	{0b0001,{S7,S5}},
	{0b1001,{S0,S6}},
};


void Stepper_init2(PinName_t pinA, PinName_t pinB,  PinName_t pinAN, PinName_t pinBN){

	//  GPIO Digital Out Initiation	
	
	myStepper2.A = pinA;
	myStepper2.B = pinB;
	myStepper2.AN = pinAN;
	myStepper2.BN = pinBN;
	
	int Stepper_pins[4] = {myStepper2.A, myStepper2.B , myStepper2.AN, myStepper2.BN};
	configure_stepper(Stepper_pins, sizeof(Stepper_pins)); 
}

/*
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
	 
	//  GPIO Digital Out Initiation
	myStepper.port1 = port1;
	myStepper.pin1  = pin1;
	myStepper.port2 = port2;
	myStepper.pin2  = pin2;
	myStepper.port3 = port3;
	myStepper.pin3  = pin3;
	myStepper.port4 = port4;
	myStepper.pin4  = pin4;
	
	//  GPIO Digital Out Initiation
	// No pull-up Pull-down , Push-Pull, Fast	
	// Pin1 ~ Port4
} 
*/


void Stepper_pinOut (uint32_t state, uint32_t mode){	
   	if (mode == FULL){         // FULL mode
		GPIO_write(myStepper2.A, (FSM_full[state].out & 0b1000) >> 3);
		GPIO_write(myStepper2.B, (FSM_full[state].out & 0b0100) >> 2);
		GPIO_write(myStepper2.AN, (FSM_full[state].out & 0b0010) >> 1);
		GPIO_write(myStepper2.BN, (FSM_full[state].out & 0b0001) >> 0);
		
	}	 
 	else if (mode == HALF){    // HALF mode
		GPIO_write(myStepper2.A, (FSM_half[state].out & 0b1000) >> 3);
		GPIO_write(myStepper2.B, (FSM_half[state].out & 0b0100) >> 2);
		GPIO_write(myStepper2.AN, (FSM_half[state].out & 0b0010) >> 1);
		GPIO_write(myStepper2.BN, (FSM_half[state].out & 0b0001) >> 0);
	}
}

//2 -> 60sec 2rev : 1 unit time delay
//divide 60[sec] divide 1000[msec] 
//divide 64*32 [step]
void Stepper_setSpeed (long whatSpeed){      // rpm [rev/min]
		step_delay = 60*1000 / (whatSpeed*64*32);  //YOUR CODE   // Convert rpm to  [msec] delay
}


void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode){
	 uint32_t state = 0;
	 myStepper2._step_num = steps;

	 for(; myStepper2._step_num > 0; myStepper2._step_num--){ // run for step size
		 delay_ms(step_delay); 				 
	   if (mode == FULL) 		 												
			state = FSM_full[state].next[direction];// YOUR CODE       // state = next state
		else if (mode == HALF) 
			state = FSM_half[state].next[direction];// YOUR CODE       // state = next state		
		Stepper_pinOut(state, mode);
   	}
}


void Stepper_stop (void){ 
    	myStepper2._step_num = 0;    
	// All pins(A,AN,B,BN) set as DigitalOut '0'
		GPIO_write(myStepper2.A, 0);
		GPIO_write(myStepper2.B, 0);
		GPIO_write(myStepper2.AN, 0);
		GPIO_write(myStepper2.BN, 0);

}


void configure_stepper(int pins[], int numPins) {
	
	for (int i = 0; i < numPins; i++) {
		// No pull-up Pull-down , Push-Pull, Fast
        GPIO_init(pins[i], OUTPUT);
        GPIO_ospeed(pins[i], EC_HIGH);
        GPIO_otype(pins[i], EC_PUSH_PULL);
        GPIO_pupd(pins[i], EC_NONE);
    }
}

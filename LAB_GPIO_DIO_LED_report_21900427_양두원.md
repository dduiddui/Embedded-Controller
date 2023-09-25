# LAB: GPIO Digital InOut
####  Date : 2023.09.23
#### Author/Partner: Duwon Yang / Yoonseok Choi
#### Github : [LINK](https://github.com/dduiddui/Embedded-Controller.git)
#### Demo Video : [LINK](https://youtu.be/K8H1wVXmPPc)
# Ⅰ. Introduction
In this lab, We create a simple program that switch multiple LEDs with pushbutton input.

We used Nucleo-F411RE to implement this program, and the library by creating a HAL driver for GPIO digital input and output control.

## Requirement
#### Hardware
+ MCU 
  + NUCLEO-F411RE
+ Actuator/Sensor/Others:
  + LEDs x 3
  + Resistor 330 ohm x 3, breadboard
#### Software
+ Keli uVision, CMSIS, EC_HAL library

## Problem 1 : Create EC_HAL library

### Procedure
Create the library directory ```\repos\EC\lib\.```

Save header library files in this directory.

Create own library for Digital_In and Out : ```ecGPIO.h, ecGPIO.c```

#### __ecRCC.h__ (provided)
```C
void	RCC_HSI_init(void);
void	RCC_GPIOA_enable(void);
void	RCC_GPIOB_enable(void);
void	RCC_GPIOC_enable(void);
void	RCC_GPIOD_enable(void);
 ```

#### __ecGPIO.h__ (provided)
```C
void GPIO_init(GPIO_TypeDef *Port, unsigned int pin, unsigned int mode);
void GPIO_write(GPIO_TypeDef *Port, unsigned int pin, unsigned int Output);
unsigned int  GPIO_read(GPIO_TypeDef *Port, unsigned int pin);
void GPIO_mode(GPIO_TypeDef* Port, unsigned int pin, unsigned int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, unsigned int pin, unsigned int speed);
void GPIO_otype(GPIO_TypeDef* Port, unsigned int pin, unsigned int type);
void GPIO_pupd(GPIO_TypeDef* Port, unsigned int pin, unsigned int pupd);
 ```

## Problem 2 : Toggle LED with Button

### Procedure
1. Create a new project under the directory  ```\repos\EC\lib\.```

+ The project name is __"LAB_GPIO_DIO_LED"__
+ Name the source file as __"LAB_GPIO_DIO_LED.c"__

2. Include library __ecGPIO.h, ecGPIO.c__ in ```\repos\EC\lib\```

3. Toggle the LED by pushing the button.
+ Push button (LED ON), Push Button (LED OFF) and repeat

### Configuration
![Alt text](image-3.png)

### Circuit diagram

![Alt text](image-4.png)

### Description with Code

+ Lab source code: [LINK](https://github.com/dduiddui/Embedded-Controller.git)


+ setup the code :
Define setup Function and Pin number.

```C
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
	

// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  		// calls RCC_GPIOC_enable()
	GPIO_mode(GPIOC, BUTTON_PIN, INPUT);  		// Set GPIOC as INPUT
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);  		// Set GPIOC as PULL_UP
	
	
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    		// calls RCC_GPIOA_enable()
	GPIO_mode(GPIOA, LED_PIN, OUTPUT);    		// Set GPIOC as OUTPUT
	GPIO_pupd(GPIOA, LED_PIN, EC_PU);     		// Set GPIOC as PULL_UP
	GPIO_otype(GPIOA, LED_PIN, EC_PUSH_PULL);   // Set GPIOC as PUSH_PULL
	GPIO_ospeed(GPIOA, LED_PIN, EC_MEDIUM);     // Set GPIOC as MEDIUM SPEED
}
```

+ main code :
as button(B1) is pressed, let a LED ON. If is not, LED OFF.
```C
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0)	GPIO_write(GPIOA, LED_PIN, HIGH);
		else 					GPIO_write(GPIOA, LED_PIN, LOW);
	}
}
```

### Discussion
1. Find out a typical solution for software debouncing and hardware debouncing

#### Software debouncing
At there, a timer can be used to perform a debouncing role.
if the input change is deemed valid for a defined period of time, perform an operation.
otherwise if deemed invalid, not perform the operation until the input stabilizes.

#### Hardware debouncing
In hardware, a low-frequency filter using resistors and capacitors serves to reduce noise. Additionally, the Schmitt trigger can be used to convert analog signals into clean digital signals with hysteresis. This allows the output to be maintained stably until the input signal exceeds a certain threshold.


2. What method of debouncing did this NUCLEO board use for the push-button(B1)?

Generally, use the method of Hardware debouncing. The push-button(B1) on the NUCLEO board has debouncing implemented in a hardware manner, through which noise and bounce effects occurring in button input are filterd out and converted into stable digital signals. This manners are achieved through the RC low-frequency filter and Schmitt trigger mentioned above.

## Problem 3 : Toggle multiLED with Button

### Procedure
1. Create a new project under the directory  ```\repos\EC\lib\.```

+ The project name is __"LAB_GPIO_DIO_multiLED"__
+ Name the source file as __"LAB_GPIO_DIO_multiLED.c"__

2. Include library __ecGPIO.h, ecGPIO.c__ in ```\repos\EC\lib\```

3. Connect 4 LEDs externally with necessary load resistors. Toggle the LED sequentially by pushing the button.
+ As button B1 is pressed, light one LED at a time, in sequence.
+ Example: LED0->LED1->...LED3->...LED0...

### Configuration
![Alt text](<led 3번-1.jpg>)

### Circuit diagram



### Description with Code

+ Lab source code: [LINK](https://github.com/dduiddui/Embedded-Controller.git)


+ setup the code :
Define setup Function and Pin number(C13, A5, A6, A7, B6). set Button Pin as PULL-UP, and LEDs as Push-Pull, Pull-up, Medium speed.

```C
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PIN_1 	5
#define LED_PIN_2 	6
#define LED_PIN_3 	7
#define LED_PIN_4 	6
#define BUTTON_PIN 13

void setup(void);
	
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
```

+ main code :
as button(B1) is pressed, the delay continues to be added within an infinite loop, and LED PIN 1-4 is turned on and off sequentially by debouncing under the condition that it exceeds a certain value(100,000).
```C
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
```


# Ⅲ. Configuration
#### Ultrasonic distance sensor
Trigger:
+ Generate a trigger pulse as PWM to the sensor
+ Pin : D10 (TIM4 CH1)
+ PWM out: 50ms period, 10us pulse-width

Echo:
+ Receive echo pulses from the ultrasonic sensor
+ Pin : D7 (Timer1 CH1)
+ Measure the distance by calculating pulse-width of the echo pulse.

USART
+ Display measured distance in [cm] on serial monitor of Tera-Term.
+ Baudrate 9600

DC Motor
+ PWM: PWM1, set 10ms of period by default
+ Pin: D11 (Timer1 CH1N)
### Circuit/Wiring Diagram
External circuit diagram that connects MCU pins to peripherals(sensor/actuator)
![Alt text](image.png)


# Ⅳ. Algorithm
### Overview
#### Mealy FSM Table

![Alt text](image-1.png)
#### Mealy State Diagram
![Alt text](image-2.png)

### Description with Code
+ Lab source code: [LINK](https://github.com/dduiddui/Embedded-Controller.git)


+ Description1
Define State/Output Form/Function and Pin, and declare multiple variables.
```c
#define S0  0
#define S1  1 
#define S2  2
#define OFF LOW
#define ON HIGH

void nextState();
void pressed();
void stateOutput();
//핀 설정
const int trigPin = 10;
const int echoPin = 7;
const int ledPin = 13;
const int pwmPin = 11;
const int btnPin = 3;

long interval = 1000;
long curtime;
long pretime;
long duration;
long distance;
int state = S0;
int ledState = 0;
int pwmState = 0;
int pwmOut = 0;
int ledOut = LOW;
int input[2] = {0, 0};
```
+ Description2

Define State table components and set arrays according to above FSM table.
```c
typedef struct {
  unsigned int next[2][2]; // nextstate = FSM[state].next[inputX][inputY] X에 의해서만 구분가능
  unsigned int vel[2][2]; // velocity = FSM[state].vel[inputX][inputY]
  unsigned int led[2][2]; // led = FSM[state].led[inputX][inputY]
} State_t;

State_t FSM[3] = {
{{{S0,S0}, {S1,S1}},{{0,0},{0,50}},{{OFF,OFF},{ON,ON}}},
{{{S1, S1},{S2, S2}},{{0,50},{0,100}},{{ON,ON},{ON,ON}}},
{{{S2, S2},{S0, S0}},{{0,100},{0,0}},{{ON,ON},{OFF,OFF}}},
};
```
+ Description3

setup led pin, button pin, motor pin, and Ultrasonic distance pin.
additionally add interrupt function (when pressed button, do pressed).
```c
void setup() {
  Serial.begin(9600);  
  pinMode(ledPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  
  pinMode(btnPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(btnPin), pressed, FALLING);
}
```
+ Description4

Generate pwm signal on the trigger pin and calculate distance using how much time it takes.

update State and output. then, write pwmOut, ledOut value to pwm, led.

finally, print input value[1] (whether face nearby), state, distance.
```c
void loop() {
  //Generate pwm signal on the trigger pin.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
 
  //Distance is calculated using how much time it takes.
  duration = pulseIn(echoPin, HIGH);
  distance = (float)duration / 58.0;


  //Calculate next state. then update State
  nextState();
  // Output State
  stateOutput();

  analogWrite(pwmPin, pwmOut); //pwm에 입력
  digitalWrite(ledPin, ledOut); //ledState에 입력

 

  Serial.print("input : ");
  Serial.print(input[1]);
  Serial.print(", state : ");
  Serial.print(state);

  Serial.print(", distance = ");
  Serial.print(distance);
  Serial.println(" [cm]");
  delay(200);
}
```
+ Description5
when the button pressed, change input[0] = 1, then apply to nextState function.

wrap up by initializing to input[0] = 0
```c
void pressed(){
  input[0] = 1;
  nextState();
  input[0] = 0;
}
```
+ Description6

when the distance is less than 10cm, update input[1] = 1

if not, update input[1] = 0

Finish and update the state
```c
void nextState(){
  if(distance < 10)
    input[1] = 1;
  else 
    input[1] = 0;
  state = FSM[state].next[input[0]][input[1]];
}
```
+ Description7

receive current time data at variable [curtime] from function millis().

if state equals 0, ledOut is updated by FSM.

if not, calculate interval time[curtime-pretime] and compare with variable [interval].

at there, if interval time[curtime-pretime] is more than [interval], update curtime to pretime, toggle ledOut, and output led.
```c
void stateOutput(){
  curtime = millis();
  pwmOut = FSM[state].vel[input[0]][input[1]];
  if(state == 0)
    ledOut = FSM[state].led[input[0]][input[1]];
  else{
    if(curtime-pretime>=interval){
      pretime = curtime;
      ledOut ^= 1;
      digitalWrite(ledPin, ledOut);
    }
  }
}
```


# Results and Analysis
### Results
At this experiment, We implemented a smart mini fan that has 3 output modes(0, 50, 100) and operates only when approached under 10cm. even more, the LED is designed to turn off or on at 1 second intervals if it is in a state where it can work when it is close under 10cm.

### Demo Video
[LINK](https://youtu.be/K8H1wVXmPPc)

### Analysis
According to 3 output modes, system operates only approached under 10cm. 
in each state, DC motor has specific velocity (0, 50, 100).
the LED is designed to turn off or on at 1 second intervals if it is in a state where it can work when it is close under 10cm.


# Reference
https://ykkim.gitbook.io/ec/ec-course/lab/lab-report-template
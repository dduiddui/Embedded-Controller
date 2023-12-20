/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : DUWON YANG
Created          : 05-03-2021
Modified         : 10-13-2023
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for PWM_Timer
/----------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "ecPWM.h"
#include "math.h"

/* PWM Configuration using PinName_t Structure */

/* PWM initialization */
// Default: 84MHz PLL, 1MHz CK_CNT, 50% duty ratio, 1msec period
void PWM_init(PinName_t pinName) {                   // 1. Setting AF, AFR, PSC

// 0. Match TIMx from  Port and Pin    
    GPIO_TypeDef *port;
    unsigned int pin;
    ecPinmap(pinName, &port, &pin);
    TIM_TypeDef *TIMx;
    int chN;
    PWM_pinmap(pinName, &TIMx, &chN);


// 1. Initialize GPIO port and pin as AF   
    GPIO_init(pinName, AF);  // AF=2
    GPIO_ospeed(pinName, EC_HIGH);
    GPIO_otype(pinName, EC_PUSH_PULL);    //if necessary
    GPIO_pupd(pinName, EC_PU);                    //if necessary

// 2. Configure GPIO AFR by Pin num.   
    //  AFR[0] for pin: 0~7,     AFR[1] for pin 8~15
    //  AFR=1 for TIM1,TIM2   AFR=2 for TIM3 etc

      if(TIMx == TIM1 | TIMx == TIM2)
         port->AFR[pin>>3] |= 0x01 << 4*(pin%8);
      else if(TIMx == TIM3 | TIMx == TIM4 | TIMx == TIM5)
         port->AFR[pin>>3]  |= 0x02 << (4*(pin%8));
      else
         port->AFR[pin>>3]  |= 0x03 << (4*(pin%8));



// 3. Initialize Timer 
    TIM_init(TIMx, 1);    // with default msec=1msec value.
		TIMx->CR1 &= ~TIM_CR1_CEN;
// 3-2. Direction of Counter

    TIMx->CR1 &= ~TIM_CR1_DIR;                          // Counting direction: 0 = up-counting, 1 = down-counting

// 4. Configure Timer Output mode as PWM
    uint32_t ccVal = TIMx->ARR / 2;  // default value  CC=ARR/2
    if (chN == 1) {
        TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;                     // Clear ouput compare mode bits for channel 1
        TIMx->CCMR1 |= 6UL << 4UL;                          // OC1M = 110 for PWM Mode 1 output on ch1. #define TIM_CCMR1_OC1M_1  (0x2UL << TIM_CCMR1_OC1M_Pos)
        TIMx->CCMR1 |= TIM_CCMR1_OC1PE;                     // Output 1 preload enable (make CCR1 value changable)
        TIMx->CCR1 = ccVal;                                 // Output Compare Register for channel 1 (default duty ratio = 50%)
        TIMx->CCER &= ~TIM_CCER_CC1P;                       // select output polarity: active high
        TIMx->CCER |= TIM_CCER_CC1E;                        // Enable output for ch1
    } else if (chN == 2) {
        TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;
        TIMx->CCMR1 |= 6UL << 12UL;
        TIMx->CCMR1 |= TIM_CCMR1_OC2PE;
        TIMx->CCR2 = ccVal;
        TIMx->CCER &= ~TIM_CCER_CC2P;
        TIMx->CCER |= TIM_CCER_CC2E;
    } else if (chN == 3) {
        TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;
        TIMx->CCMR2 |= 6UL << 4UL;
        TIMx->CCMR2 |= TIM_CCMR2_OC3PE;
        TIMx->CCR3 = ccVal;
        TIMx->CCER &= ~TIM_CCER_CC3P;
        TIMx->CCER |= TIM_CCER_CC3E;
    } else if (chN == 4) {
        TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;
        TIMx->CCMR2 |= 6UL << 12UL;
        TIMx->CCMR2 |= TIM_CCMR2_OC4PE;
        TIMx->CCR4 = ccVal;
        TIMx->CCER &= ~TIM_CCER_CC4P;
        TIMx->CCER |= TIM_CCER_CC4E;
    }

// 5. Enable Timer Counter
    // For TIM1 ONLY
    if (TIMx == TIM1)
        TIMx->BDTR |= TIM_BDTR_MOE;                    // Main output enable (MOE): 0 = Disable, 1 = Enable
    // Enable timers
    TIMx->CR1 |= TIM_CR1_CEN;                          // Enable counter

}

/* PWM PERIOD SETUP */
// allowable range for msec:  1~2,000
void PWM_period_ms(PinName_t pinName, uint32_t msec) {

// 0. Match TIMx from  Port and Pin
    GPIO_TypeDef *port;
    unsigned int pin;
    ecPinmap(pinName, &port, &pin);
    TIM_TypeDef *TIMx;
    int chN;
    PWM_pinmap(pinName, &TIMx, &chN);


// 1. Set Counter Period in msec
    TIM_period_ms(TIMx, msec);

}

// allowable range for msec:  1~2,000
void PWM_period(PinName_t pinName,  uint32_t msec){
   PWM_period_ms(pinName,  msec);
}


// allowable range for usec:  1~1,000
void PWM_period_us(PinName_t pinName, uint32_t usec) {

// 0. Match TIMx from  Port and Pin    
    GPIO_TypeDef *port;
    unsigned int pin;
    ecPinmap(pinName, &port, &pin);
    TIM_TypeDef *TIMx;
    int chN;
    PWM_pinmap(pinName, &TIMx, &chN);

// 1. Set Counter Period in usec
    TIM_period_us(TIMx, usec);
}

/* DUTY RATIO SETUP */
// High Pulse width in msec
void PWM_pulsewidth(PinName_t pinName, uint32_t pulse_width_ms) {
// 0. Match TIMx from  Port and Pin    
    GPIO_TypeDef *port;
    unsigned int pin;
    ecPinmap(pinName, &port, &pin);
    TIM_TypeDef *TIMx;
    int chN;
    PWM_pinmap(pinName, &TIMx, &chN);

// 1. Declaration System Frequency and Prescaler
    uint32_t fsys = 0;
    uint32_t psc = TIMx->PSC;

// 2. Check System CLK: PLL or HSI
    if ((RCC->CFGR & RCC_CFGR_SW_PLL) == RCC_CFGR_SW_PLL) fsys = 84000;  // for msec 84MHz/1000 [msec]
    else if ((RCC->CFGR & RCC_CFGR_SW_HSI) == RCC_CFGR_SW_HSI) fsys = 16000;

// 3. Configure prescaler PSC
    float fclk = fsys / (psc + 1);                                    // fclk=fsys/(psc+1);
    uint32_t value = (float)pulse_width_ms * fclk - 1;                    // pulse_width_ms *fclk - 1;

    switch (chN) {
        case 1:TIMx->CCR1 = value;break;
        case 2:TIMx->CCR2 = value;break;
        case 3:TIMx->CCR3 = value;break;
        case 4:TIMx->CCR4 = value;break;
        default:break;
    }
}

// High Pulse width in msec
void PWM_pulsewidth_ms(PinName_t pinName, uint32_t pulse_width_ms) {
    PWM_pulsewidth(pinName, pulse_width_ms);
}

// High Pulse width in usec
void PWM_pulsewidth_us(PinName_t pinName, uint32_t pulse_width_us) {
// 0. Match TIMx from  Port and Pin    
    GPIO_TypeDef *port;
    unsigned int pin;
    ecPinmap(pinName, &port, &pin);
    TIM_TypeDef *TIMx;
    int chN;
    PWM_pinmap(pinName, &TIMx, &chN);

// 1. Declaration system frequency and prescaler
    uint32_t fsys = 0;
    uint32_t psc = TIMx->PSC;


// 2. Check System CLK: PLL or HSI
    if ((RCC->CFGR & RCC_CFGR_SW_PLL) == RCC_CFGR_SW_PLL) fsys = 84;  // for msec 84MHz/1000000 [usec]
    else if ((RCC->CFGR & RCC_CFGR_SW_HSI) == RCC_CFGR_SW_HSI) fsys = 16;


// 3. Configure prescaler PSC
    float fclk = fsys / (psc + 1);                    // fclk=fsys/(psc+1);
    uint32_t value = (float)pulse_width_us * fclk - 1;                    // pulse_width_ms *fclk - 1;

    switch (chN) {
        case 1:TIMx->CCR1 = value;break;
        case 2:TIMx->CCR2 = value;break;
        case 3:TIMx->CCR3 = value;break;
        case 4:TIMx->CCR4 = value;break;
        default:break;
    }
}

// Dutry Ratio from 0 to 1 
void PWM_duty(PinName_t pinName, float duty) {

// 0. Match TIMx from  Port and Pin    
    GPIO_TypeDef *port;
    unsigned int pin;
    ecPinmap(pinName, &port, &pin);
    TIM_TypeDef *TIMx;
    int chN;
    PWM_pinmap(pinName, &TIMx, &chN);


// 1. Configure prescaler PSC
    float value = (TIMx->ARR + 1) * duty + 1;      // (ARR+1)*dutyRatio + 1

    if (chN == 1) { TIMx->CCR1 = (uint32_t)value; }          //set channel
    else if (chN == 2) { TIMx->CCR2 = (uint32_t)value; }
    else if (chN == 3) { TIMx->CCR3 = (uint32_t)value; }
    else if (chN == 4) { TIMx->CCR4 = (uint32_t)value; }
}

// DO NOT MODIFY HERE
void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN) {
    GPIO_TypeDef *port;
    unsigned int pin;
    ecPinmap(pinName, &port, &pin);


    if(port == GPIOA) {
      switch(pin){
         case 0 : *TIMx = TIM2; *chN   = 1; break;
         case 1 : *TIMx = TIM2; *chN = 2; break;
         case 5 : *TIMx = TIM2; *chN = 1; break;
         case 6 : *TIMx = TIM3; *chN = 1; break;
         //case 7: TIMx = TIM1; *chN = 1N; break;
         case 8 : *TIMx = TIM1; *chN = 1; break;
         case 9 : *TIMx = TIM1; *chN = 2; break;
         case 10: *TIMx = TIM1; *chN = 3; break;
         case 15: *TIMx = TIM2; *chN = 1; break;
         default: break;
      }         
   }
   else if(port == GPIOB) {
      switch(pin){
         //case 0: TIMx = TIM1; *chN = 2N; break;
         //case 1: TIMx = TIM1; *chN = 3N; break;
         case 3 : *TIMx = TIM2; *chN = 2; break;
         case 4 : *TIMx = TIM3; *chN = 1; break;
         case 5 : *TIMx = TIM3; *chN = 2; break;
         case 6 : *TIMx = TIM4; *chN = 1; break;
         case 7 : *TIMx = TIM4; *chN = 2; break;
         case 8 : *TIMx = TIM4; *chN = 3; break;
         case 9 : *TIMx = TIM4; *chN = 4; break;
         case 10: *TIMx = TIM2; *chN = 3; break;     
         default: break;
      }
   }
   else if(port == GPIOC) {
      switch(pin){
         case 6 : *TIMx = TIM3; *chN = 1; break;
         case 7 : *TIMx = TIM3; *chN = 2; break;
         case 8 : *TIMx = TIM3; *chN = 3; break;
         case 9 : *TIMx = TIM3; *chN = 4; break;
         default: break;
      }
   }
    // TIM5 needs to be added, if used.
}
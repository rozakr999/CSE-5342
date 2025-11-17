//Final project for Embedded2
//PWM setup
//PA9 as PWM output using TIM1_CH1

//using STM32F446RE Nucleo board
//using SG90 servo motor

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "servomotor.h"

//DEFINES
#define PWM_GPIO_PORT GPIOA
#define PWM_GPIO_PIN 9 //PA9 = TIM1_CH2

void delay_Ms(int n)
{
	int i;
	for (; n > 0; n--)
	{
		for (i = 0; i < 1067; i++); //~1ms delay at 16MHz
	}
}

//pwm config using tim1_ch1 
//pwm frequency 50 Hz
void initPwm(void)
{
	//enable clocks for GPIOA and GPIOB and TIM1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	(void)RCC->AHB1ENR; //dummy read
	
	//PA8 as alternate function
	PWM_GPIO_PORT->MODER &= ~(3U << (2*PWM_GPIO_PIN));
	PWM_GPIO_PORT->MODER |= (2U << (2*PWM_GPIO_PIN)); //AF mode
	PWM_GPIO_PORT->AFR[PWM_GPIO_PIN >> 3] &= ~(0xFU << (4*(PWM_GPIO_PIN & 7)));
	PWM_GPIO_PORT->AFR[PWM_GPIO_PIN >> 3] |= (1U << (4*(PWM_GPIO_PIN & 7))); //AF1 FOR TIM1?

	//CONFIGURE TIM1 FOR PWM
	// base clock is 16 MHz (default)
	TIM1->PSC = 15; // 15-> 16 MHz // 16 = 1 MHz tick (1us)
	TIM1->ARR = 20000 - 1; //2000 -> 20,000 us = 20ms (50 Hz)
	TIM1->CCR1 = 1500; //default pulse width 1.5ms (neutral posititon)
	
	//configure channel 1: pwm mode 1, preload enable
	TIM1->CCMR1 &= ~(TIM_CCMR1_OC2M);
	TIM1->CCMR1 |= (6U << TIM_CCMR1_OC2M_Pos); // PWM MODE 1
	TIM1->CCMR1 |= TIM_CCMR1_OC2PE; //PRELOAD ENABLE
	
	TIM1->CCER |= TIM_CCER_CC2E; // ENABLE CHANNEL 2 OUTPUT
	TIM1->BDTR |= TIM_BDTR_MOE; // ENABLE MAIN OUTPUT
	TIM1->CR1 |= TIM_CR1_ARPE; // ENEABLE ARPE
	TIM1->EGR |= TIM_EGR_UG; // GENERATE UPDATE EVENT  OT LOAD THE PRESCALER VALUE
	TIM1->CR1 |= TIM_CR1_CEN; // START TIMER COUNTER
	
}

//set sevo pulde width directly in microseconds
void setServoPulse(uint16_t pulseWidth_us)
{
	//keep pu;se width in safe range
	if (pulseWidth_us < 500) pulseWidth_us = 500;
	if (pulseWidth_us > 2500) pulseWidth_us = 2500;
	TIM1->CCR2 = pulseWidth_us; //write pulse width to timers compar register
}

// convert angle (0-180) pulse width
//send to servo
void setServoAngle(uint8_t angle)
{
	if (angle > 180) angle = 180; //angle in valid range
	//mapping angle (0D-500us) (180D-2500us)
	uint16_t pulse = 500 + ((uint32_t) angle * 2000) / 180; // 500-2500us
	setServoPulse(pulse);
}

void sweepServo(void)
{
	//sweep servo 0 -> 180
		for (uint8_t angle = 0; angle <= 180; angle += 5) // angle per step; lower number for slower sweep
		{
			setServoAngle(angle);
			delay_Ms(50); //delay per step; lower number for smaller delay = faster
		}
		delay_Ms(1000); //pause for 1 seconds
		
		//sweep back 180 -> 0
		for (uint8_t angle = 180; angle > 0; angle -=5)
		{
			setServoAngle(angle);
			delay_Ms(50);
		}
		delay_Ms(1000); 
	}
	
/*
int main(void)
{
	SystemInit();
	SystemCoreClockUpdate();
	//uint32_t sysclk = SystemCoreClock;
	initPwm();
	
	while (1)
	{
		sweepServo();
	}
}
*/



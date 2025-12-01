//Final project for Embedded2
//low level passive buzzer setup
//PA0 as digital output

//using STM32F446RE Nucleo board


#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "buzzer.h"


//DEFINES
#define BUZZER_PORT GPIOA
#define BUZZER_PIN 0 //PA0 = TIM2_CH1

void delayms(int n)
{
	int i;
	for (; n > 0; n--)
	{
		for (i = 0; i < 1067; i++); //~1ms delay at 16MHz
	}
}
//Initialize PA0 as a digital ouput
void buzzerInit(void)
{
	//enable port clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//set PA0 to output mode
	BUZZER_PORT->MODER &= ~(3U << (2*BUZZER_PIN)); 
	BUZZER_PORT->MODER |= (1U << (2*BUZZER_PIN));
	//default output high
	BUZZER_PORT->ODR |= (1U << BUZZER_PIN);
}
	//configure TIM2 CH1 for pwm generation ad given freq
void initBuzzerPWM(uint32_t frequency_hz)
{
	//enable clock for TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	//configure PA0 as AF1 (TIM2_CH1)
	BUZZER_PORT->MODER &= ~(3U << (2*BUZZER_PIN));
	BUZZER_PORT->MODER |= (2U << (2*BUZZER_PIN)); //AF mode
	BUZZER_PORT->AFR[0] &= ~(0xF << (4*BUZZER_PIN));
	BUZZER_PORT->AFR[0] |= (1U << (4*BUZZER_PIN));
	
	//Configure TIM2 bas freq
	//base clock = 16 MHz
	TIM2->PSC = 16 - 1; // 1MHz timer tick
	//defines pwm freq
	uint32_t arr = (1000000 / frequency_hz) - 1; // ARR = period in us
	TIM2->ARR = arr;
	TIM2->CCR1 = arr / 2; //50% duty cycle for square wave
	
	//pwm mode 1 on CH1
	TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM2->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos); // pwm mode 1
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE; //enable preload
	
	TIM2->CR1 |= TIM_CR1_ARPE; //enable auto reload preload
	TIM2->EGR |= TIM_EGR_UG; //update event
	//ensure channel disabled until told
	TIM2->CCER &= ~TIM_CCER_CC1E; 
	TIM2->CR1 &= ~TIM_CR1_CEN; 
}

//turn buzzer on
void buzzerOn(void)
{
	TIM2->EGR |= TIM_EGR_UG; //update registers
	TIM2->CCER |= TIM_CCER_CC1E; //enable CH1 output
	TIM2->CR1 |= TIM_CR1_CEN; //start timer
}

//turn buzzer off
void buzzerOff(void)
{
	TIM2->CCER &= ~TIM_CCER_CC1E; // disable channel output
	TIM2->CR1 &= ~TIM_CR1_CEN; //stop timer
}
//generate a tone at freq for duration
void buzzerBeep(uint32_t frequency_hz, uint32_t duration_ms)
{
	initBuzzerPWM(frequency_hz); //configure pwm
	buzzerOn(); //set tone
	delayms(duration_ms); //wait
	buzzerOff(); //stop tone
}
	

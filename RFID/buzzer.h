#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

//DEFINES
#define BUZZER_PORT GPIOA
#define BUZZER_PIN 0 //PA0 = TIM2_CH1

void delayms(int n);
void buzzerInit(void);
void initBuzzerPWM(uint32_t frequency_hz);
void buzzerOn(void);
void buzzerOff(void);
void buzzerBeep(uint32_t frequency_hz, uint32_t duration_ms);


#endif

#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

//DEFINES
#define PWM_GPIO_PORT GPIOA
#define PWM_GPIO_PIN 9 //PA9 = TIM1_CH2

void delay_Ms(int n);
void initPwm(void);
void setServoPulse(uint16_t pulseWidth_us);
void setServoAngle(uint8_t angle);
void sweepServo(void);


#endif

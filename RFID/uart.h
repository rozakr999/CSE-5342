#ifndef UART_H
#define UART_H

#include "stm32f446xx.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define MAX_CHARS 80

typedef struct _USER_DATA
{
	char buffer[MAX_CHARS+1];
} USER_DATA;
	
void initUsart2(void);
void putcUart(char c);
void putsUart(char* str);
char getcUart(void);
void getData(USER_DATA* data);

#endif // UART_H

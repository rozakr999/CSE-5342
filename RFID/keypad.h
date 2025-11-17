#ifndef KEYPAD_H
#define KEYPAD_H

#include "stm32f446xx.h"
#include <stdint.h>

/* ===== 4x4 Keypad Pin Mapping =====
   Keypad Pin 8 → R1 → PB12
   Keypad Pin 7 → R2 → PB13
   Keypad Pin 6 → R3 → PB14
   Keypad Pin 5 → R4 → PB15
   Keypad Pin 4 → C1 → PC6
   Keypad Pin 3 → C2 → PA11
   Keypad Pin 2 → C3 → PC8
   Keypad Pin 1 → C4 → PA12
*/

/* ---- Row Pins ---- */
#define KP_R1_PORT GPIOB
#define KP_R1_PIN  12
#define KP_R2_PORT GPIOB
#define KP_R2_PIN  13
#define KP_R3_PORT GPIOB
#define KP_R3_PIN  14
#define KP_R4_PORT GPIOB
#define KP_R4_PIN  15

/* ---- Column Pins ---- */
#define KP_C1_PORT GPIOC
#define KP_C1_PIN  6
#define KP_C2_PORT GPIOA
#define KP_C2_PIN  11
#define KP_C3_PORT GPIOC
#define KP_C3_PIN  8
#define KP_C4_PORT GPIOA
#define KP_C4_PIN  12

/* ---- Function Prototypes ---- */

void keypad_init(void);


char keypad_scan(void);


uint8_t keypad_read_code(char *buf, uint8_t len, uint32_t timeout_ms);

#endif /* KEYPAD_H */

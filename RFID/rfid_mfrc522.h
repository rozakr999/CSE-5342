#ifndef RFID_MFRC522_H
#define RFID_MFRC522_H

#include "stm32f446xx.h"
#include <stdint.h>


// SPI1
#define SCK_PORT   GPIOA
#define SCK_PIN    5    // PA5, D13
#define MISO_PORT  GPIOA
#define MISO_PIN   6    // PA6, D12
#define MOSI_PORT  GPIOA
#define MOSI_PIN   7    // PA7, D11

// MFRC522 control pins
#define CS_PORT    GPIOB
#define CS_PIN     6    // PB6, D10 (chip select)
#define RST_PORT   GPIOB
#define RST_PIN    5    // PB5, D4  (reset)


#define PIN_MASK(p)             (1U << (p))
#define GPIO_SET(port,pin)      ((port)->BSRR = PIN_MASK(pin))
#define GPIO_CLR(port,pin)      ((port)->BSRR = PIN_MASK(pin) << 16)

static inline void cs_low(void)  { GPIO_CLR(CS_PORT, CS_PIN); }
static inline void cs_high(void) { GPIO_SET(CS_PORT, CS_PIN); }
static inline void rst_low(void) { GPIO_CLR(RST_PORT, RST_PIN); }
static inline void rst_high(void){ GPIO_SET(RST_PORT, RST_PIN); }


#define CommandReg    0x01
#define TxControlReg  0x14
#define VersionReg    0x37

#define SoftResetCmd  0x0F


void     delay_ms(volatile uint32_t ms);


void     enable_gpio_spi1_clocks(void);
void     config_spi1_pins(void);
void     spi1_init_mode0(void);
uint8_t  spi1_txrx(uint8_t v);

/* mfrc522 primitives */
void     mfrc522_hw_reset(void);
void     mfrc522_soft_reset(void);
uint8_t  mfrc522_read(uint8_t reg);
void     mfrc522_write(uint8_t reg, uint8_t val);
void     mfrc522_antenna_on(void);

#endif 

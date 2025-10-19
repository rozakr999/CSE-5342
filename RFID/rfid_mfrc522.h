#ifndef RFID_MFRC522_H
#define RFID_MFRC522_H

#include "stm32f446xx.h"
#include <stdint.h>

/* -------- SPI1 pins -------- */
#define SCK_PORT   GPIOA
#define SCK_PIN    5
#define MISO_PORT  GPIOA
#define MISO_PIN   6
#define MOSI_PORT  GPIOA
#define MOSI_PIN   7

/* -------- MFRC522 control pins -------- */
#define CS_PORT    GPIOB
#define CS_PIN     6
#define RST_PORT   GPIOB
#define RST_PIN    5

/* -------- GPIO helpers -------- */
#define PIN_MASK(p)             (1U << (p))
#define GPIO_SET(port,pin)      ((port)->BSRR = PIN_MASK(pin))
#define GPIO_CLR(port,pin)      ((port)->BSRR = PIN_MASK(pin) << 16)

static inline void cs_low(void)  { GPIO_CLR(CS_PORT, CS_PIN); }
static inline void cs_high(void) { GPIO_SET(CS_PORT, CS_PIN); }
static inline void rst_low(void) { GPIO_CLR(RST_PORT, RST_PIN); }
static inline void rst_high(void){ GPIO_SET(RST_PORT, RST_PIN); }

/* -------- MFRC522 registers used -------- */
#define CommandReg       0x01
#define CommIrqReg       0x04
#define DivIrqReg        0x05
#define ErrorReg         0x06
#define FIFODataReg      0x09
#define FIFOLevelReg     0x0A
#define BitFramingReg    0x0D
#define ModeReg          0x11
#define TxModeReg        0x12
#define RxModeReg        0x13
#define TxControlReg     0x14
#define TxASKReg         0x15
#define RFCfgReg         0x26
#define TModeReg         0x2A
#define TPrescalerReg    0x2B
#define TReloadRegH      0x2C
#define TReloadRegL      0x2D
#define CRCResultRegH    0x21
#define CRCResultRegL    0x22
#define VersionReg       0x37

/* -------- MFRC522 commands -------- */
#define PCD_Idle         0x00
#define PCD_CalcCRC      0x03
#define PCD_Transceive   0x0C

/* -------- PICC (card) commands -------- */
#define PICC_REQA        0x26   /* 7-bit command */
#define PICC_SEL_CL1     0x93

/* -------- Other constants -------- */
#define SoftResetCmd     0x0F


//GPIO
#define LED_GREEN_PORT GPIOB
#define LED_GREEN_PIN  0

#define LED_RED_PORT   GPIOC
#define LED_RED_PIN    0

#define LED_YELLOW_PORT GPIOA
#define LED_YELLOW_PIN  10

#define LED_BLUE_PORT GPIOA
#define LED_BLUE_PIN  8   // D7 = PA8  (NOT PA7)



//LED control macros
#define LED_ON(port,pin)    ((port)->BSRR = (1U << (pin)))
#define LED_OFF(port,pin)   ((port)->BSRR = (1U << ((pin) + 16)))


/* -------- Prototypes -------- */
void     delay_ms(volatile uint32_t ms);

void     enable_gpio_spi1_clocks(void);
void     config_spi1_pins(void);
void     spi1_init_mode0(void);
uint8_t  spi1_txrx(uint8_t v);

void     mfrc522_hw_reset(void);
void     mfrc522_soft_reset(void);
uint8_t  mfrc522_read(uint8_t reg);
void     mfrc522_write(uint8_t reg, uint8_t val);
void     mfrc522_antenna_on(void);


//led control
void leds_init(void);

#endif /* RFID_MFRC522_H */

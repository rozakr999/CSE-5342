#include "rfid_mfrc522.h"
#define SPI_MODE 0

// Simple blocking delay 
void delay_ms(volatile uint32_t ms)
{
    while (ms--) {
        for (volatile uint32_t i = 0; i < 16000; i++) __NOP();
    }
}

// Enable clocks for GPIOA, GPIOB, and SPI1
void enable_gpio_spi1_clocks(void)
{
    RCC->AHB1ENR |= (1U << 0) | (1U << 1);// GPIOA and GPIOB
    RCC->APB2ENR |= (1U << 12); // SPI1
}

//pin configuration for SPI1 
static void gpio_mode(GPIO_TypeDef *port, uint8_t pin, uint32_t mode01)
{
    port->MODER &= ~(3U << (pin*2)); // Clear mode bits
    port->MODER |=  (mode01 << (pin*2)); // Set mode bits
}

static void gpio_af(GPIO_TypeDef *port, uint8_t pin, uint8_t af)
{
    volatile uint32_t *afr = (pin < 8) ? &port->AFR[0] : &port->AFR[1]; // AFR[0] for pins 0-7, AFR[1] for pins 8-15
    uint8_t shift = (pin & 7U) * 4U; // Each pin uses 4 bits in AFR
    *afr &= ~(0xFU << shift);
    *afr |=  ((uint32_t)af << shift);
}


static void gpio_highspeed(GPIO_TypeDef *port, uint8_t pin)
{
    port->OTYPER  &= ~PIN_MASK(pin);   // Push-pull             
    port->PUPDR   &= ~(3U << (pin*2)); // No pull-up, pull-down
    port->OSPEEDR |=  (3U << (pin*2)); // High speed

}

void config_spi1_pins(void)
{
    // SCK, MISO, MOSI pins as Alternate Function
    gpio_mode(SCK_PORT,  SCK_PIN,  2); // AF mode
    gpio_mode(MISO_PORT, MISO_PIN, 2); // AF mode
    gpio_mode(MOSI_PORT, MOSI_PIN, 2); // AF mode

    // CS and RST pins as General Purpose Output
    gpio_mode(CS_PORT,   CS_PIN,   1); // Output mode
    gpio_mode(RST_PORT,  RST_PIN,  1); // Output mode

    // Set Alternate Function to AF5 (SPI1)
    gpio_af(SCK_PORT,  SCK_PIN,  5);
    gpio_af(MISO_PORT, MISO_PIN, 5);
    gpio_af(MOSI_PORT, MOSI_PIN, 5);

    // Configure pins for high speed
    gpio_highspeed(SCK_PORT,  SCK_PIN);
    gpio_highspeed(MISO_PORT, MISO_PIN);
    gpio_highspeed(MOSI_PORT, MOSI_PIN);
    gpio_highspeed(CS_PORT,   CS_PIN);
    gpio_highspeed(RST_PORT,  RST_PIN);

    // Set CS high (deselect) and RST high (not in reset)
    cs_high();
    rst_high();
}



void spi1_init_mode0(void)
{
    SPI1->CR1 = 0; // disable SPI before configuring

    // Base config: master, slow prescaler, software NSS high
    uint32_t cr1 = (1U<<2) | (7U<<3) | (1U<<9) | (1U<<8); // MSTR, BR=/256, SSM, SSI

#if (SPI_MODE == 0)
    // CPOL=0, CPHA=0  (Mode 0)
#elif (SPI_MODE == 1)
    // CPOL=0, CPHA=1  (Mode 1)
    cr1 |= (1U<<0);
#elif (SPI_MODE == 2)
    // CPOL=1, CPHA=0  (Mode 2)
    cr1 |= (1U<<1);
#elif (SPI_MODE == 3)
    // CPOL=1, CPHA=1  (Mode 3)
    cr1 |= (1U<<1) | (1U<<0);
#endif

    SPI1->CR1 = cr1;
    SPI1->CR2 = (7U<<8) | (1U<<12);   // 8-bit frames, FRXTH=1
    SPI1->CR1 |= (1U<<6);             // Enable SPI
}

uint8_t spi1_txrx(uint8_t v)
{
    while (!(SPI1->SR & (1U<<1))) { }        // TXE
    *(volatile uint8_t *)&SPI1->DR = v;      // write 8-bit
    while (!(SPI1->SR & (1U<<0))) { }        // RXNE
    return *(volatile uint8_t *)&SPI1->DR;   // read 8-bit
}

// Perform a hardware reset of the MFRC522
void mfrc522_hw_reset(void)
{
    rst_low();          // pull reset pin low
    delay_ms(50);       // small delay
    rst_high();         // release reset
    delay_ms(50);       // wait for the chip to come up
}

// Perform a software reset of the MFRC522
uint8_t mfrc522_read(uint8_t reg)
{
    // MFRC522 read command byte = (reg << 1) | 0x80
    cs_low();
    spi1_txrx(((reg << 1) & 0x7E) | 0x80); // Send address with read bit
    uint8_t val = spi1_txrx(0x00);
    cs_high();
    return val;
}

// Write a value to a MFRC522 register
void mfrc522_write(uint8_t reg, uint8_t val)
{
    // MFRC522 write command byte = (reg << 1) & 0x7E
    cs_low();
    spi1_txrx((reg << 1) & 0x7EU);
    spi1_txrx(val);
    cs_high();
}

void mfrc522_soft_reset(void)
{
    mfrc522_write(CommandReg, SoftResetCmd);
    delay_ms(50); // Wait for the reset to complete
}

// Turn on the MFRC522 antenna
void mfrc522_antenna_on(void)
{
    uint8_t v = mfrc522_read(TxControlReg); // Read current value
    mfrc522_write(TxControlReg, v | 0x03); // Set bits 0 and 1 to turn on antenna
}

//Main function
int main(void)
{
    enable_gpio_spi1_clocks();
    config_spi1_pins();
    spi1_init_mode0();

    mfrc522_hw_reset();
    mfrc522_soft_reset();
//    mfrc522_antenna_on();
    volatile uint8_t g_ver = 0;
for (;;) {
    g_ver = mfrc522_read(VersionReg); // continuously read version register
    delay_ms(50);                     // short delay
}
}


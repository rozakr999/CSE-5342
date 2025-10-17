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

// Send bytes in 'tx' and receive into 'rx'.
// bitFraming: lower 3 bits = number of valid bits in last transmit byte (0..7).
static uint8_t mfrc522_transceive(const uint8_t *tx, uint8_t txLen,
                                  uint8_t *rx, uint8_t *rxLen,
                                  uint8_t bitFraming)
{
    mfrc522_write(CommandReg, PCD_Idle);
    mfrc522_write(CommIrqReg, 0x7F);         // clear all IRQs
    mfrc522_write(FIFOLevelReg, 0x80);       // flush FIFO

    for (uint8_t i=0;i<txLen;i++) mfrc522_write(FIFODataReg, tx[i]);
    mfrc522_write(BitFramingReg, bitFraming & 0x07); // set Tx last bits (0..7)

    mfrc522_write(CommandReg, PCD_Transceive);
    // StartSend=1
    uint8_t v = mfrc522_read(BitFramingReg);
    mfrc522_write(BitFramingReg, v | 0x80);

    // wait for RxIRq(0x20) or IdleIrq(0x10)
    uint16_t to = 3000;
    while (to--) {
        uint8_t irq = mfrc522_read(CommIrqReg);
        if (irq & 0x30) break;
    }

    // Stop sending
    v = mfrc522_read(BitFramingReg);
    mfrc522_write(BitFramingReg, v & ~0x80);

    if (!to) return 1;                    // timeout
    if (mfrc522_read(ErrorReg) & 0x13)    // BufferOvfl | ParityErr | ProtocolErr
        return 2;

    uint8_t n = mfrc522_read(FIFOLevelReg);
    if (n > *rxLen) n = *rxLen;
    for (uint8_t i=0;i<n;i++) rx[i] = mfrc522_read(FIFODataReg);
    *rxLen = n;
    return 0;
}

static uint8_t picc_request_a(uint8_t atqa[2])
{
    uint8_t cmd = PICC_REQA;
    uint8_t rxLen = 2;

    uint8_t st = mfrc522_transceive(&cmd, 1, atqa, &rxLen, 7);  // 7-bit frame
    if (st)                return st;          // timeout / proto error
    if (rxLen != 2)        return 3;           // not enough data
    if (atqa[0]==0 && atqa[1]==0) return 4;    // bogus/no card

    return 0;                                   // valid ATQA
}



void mfrc522_init_14443a(void)
{
    // ~1000us timeout, auto timer
    mfrc522_write(TModeReg,      0x80);   // TAuto=1
    mfrc522_write(TPrescalerReg, 0xA9);   // ~40 kHz
    mfrc522_write(TReloadRegH,   0x03);   // 0x03E8 = 1000
    mfrc522_write(TReloadRegL,   0xE8);

    // 100% ASK
    mfrc522_write(TxASKReg,      0x40);

    // Receiver gain high (better range)
    mfrc522_write(RFCfgReg,      0x70);

    // CRC preset / TxWaitRF default
    mfrc522_write(ModeReg,       0x3D);

    // No partial-bit framing
    mfrc522_write(BitFramingReg, 0x00);

    // RF on
    mfrc522_antenna_on();
}




static void spi_set_mode(uint8_t mode) {
    uint32_t cr1 = (1U<<2) | (7U<<3) | (1U<<9) | (1U<<8); // MSTR,/256,SSM,SSI
    if (mode & 0x02) cr1 |= (1U<<1); // CPOL
    if (mode & 0x01) cr1 |= (1U<<0); // CPHA
    SPI1->CR1 = 0;
    SPI1->CR1 = cr1;
    SPI1->CR2 = (7U<<8) | (1U<<12);
    SPI1->CR1 |= (1U<<6);
}

// Main function
int main(void)
{
    // enable_gpio_spi1_clocks();
    // config_spi1_pins();
    // spi1_init_mode0();

    // mfrc522_hw_reset();
    // mfrc522_soft_reset();

    
    // mfrc522_init_14443a();
    // //volatile uint8_t ver = mfrc522_read(VersionReg);
    enable_gpio_spi1_clocks();
    config_spi1_pins();              // CS pin becomes output here

    /* --- CS sanity check: add these lines --- */
    volatile uint32_t cs_idle_idr, cs_low_idr, cs_high_idr;
    volatile uint32_t cs_idle_odr, cs_low_odr, cs_high_odr;

    cs_high();  // idle high
   cs_idle_idr = (CS_PORT->IDR >> CS_PIN) & 1U;   // expect 1
    cs_idle_odr = (CS_PORT->ODR >> CS_PIN) & 1U;   // expect 1

    cs_low();   // assert low
    cs_low_idr  = (CS_PORT->IDR >> CS_PIN) & 1U;   // expect 0
    cs_low_odr  = (CS_PORT->ODR >> CS_PIN) & 1U;   // expect 0

    cs_high();  // back high
    cs_high_idr = (CS_PORT->IDR >> CS_PIN) & 1U;   // expect 1
    cs_high_odr = (CS_PORT->ODR >> CS_PIN) & 1U;   // expect 1
    /* --- end CS sanity check --- */

    spi1_init_mode0();
    mfrc522_hw_reset();
    mfrc522_soft_reset();
    mfrc522_init_14443a();

   
    volatile uint8_t txc = mfrc522_read(TxControlReg);   // expect (txc & 0x03) == 0x03

    // Should read 0x91 or 0x92
    volatile uint8_t g_ver = mfrc522_read(VersionReg);

    // Watch these in the debugger
    volatile uint8_t st = 0;
    volatile uint8_t card_present = 0;
    volatile uint8_t atqa[2] = {0,0};

    for (;;) {
        st = picc_request_a((uint8_t*)atqa);
        card_present = (st == 0);    
        delay_ms(100);
    }
}


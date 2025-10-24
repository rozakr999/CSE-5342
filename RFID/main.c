#include "rfid_mfrc522.h"
#include "keypad.h"

#define SPI_MODE 0

// Simple blocking delay 
void delay_ms(volatile uint32_t ms)
{
    while (ms--) {
        for (volatile uint32_t i = 0; i < 16000; i++) __NOP();
    }
}

/////////////////////////////////////ADDING GPIOS/////////////////////////////////////
void enable_gpio_spi1_clocks(void)
{
    // GPIOA (bit 0), GPIOB (bit 1), GPIOC (bit 2)
    RCC->AHB1ENR |= (1U << 0) | (1U << 1) | (1U << 2);
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
    uint8_t rx[4] = {0};
    uint8_t rxLen = sizeof(rx);

    uint8_t st = mfrc522_transceive(&cmd, 1, rx, &rxLen, 7);  // 7-bit frame
    if (st)        return st;
    if (rxLen < 2) return 3;

    atqa[0] = rx[0];
    atqa[1] = rx[1];
    return 0;
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


static void mfrc522_crc(const uint8_t *data, uint8_t len, uint8_t out[2])
{
    mfrc522_write(CommandReg, PCD_Idle);
    mfrc522_write(DivIrqReg, 0x04);      // clear CRCIRq
    mfrc522_write(FIFOLevelReg, 0x80);   // flush FIFO
    for (uint8_t i=0; i<len; i++) mfrc522_write(FIFODataReg, data[i]);
    mfrc522_write(CommandReg, PCD_CalcCRC);

    // wait CRCIRq
    for (uint16_t t=0; t<5000; t++) {
        if (mfrc522_read(DivIrqReg) & 0x04) break;
    }
    out[0] = mfrc522_read(CRCResultRegL);
    out[1] = mfrc522_read(CRCResultRegH);
}


// Convert 4-byte UID to hex string (e.g., "DE AD BE EF")
static void uid_to_hex(const uint8_t uid[4], char out[3*4])
{
    const char* h = "0123456789ABCDEF";
    for (int i = 0; i < 4; ++i)
     {
        out[i*3 + 0] = h[(uid[i] >> 4) & 0xF];
        out[i*3 + 1] = h[uid[i] & 0xF];
        out[i*3 + 2] = (i == 3) ? '\0' : ' ';
    }
}



/* Anticollision (cascade level 1) → returns 4-byte UID */
static uint8_t picc_anticoll_cl1(uint8_t uid[4])
{
    uint8_t cmd[2] = { PICC_SEL_CL1, 0x20 };  // NVB=0x20
    uint8_t rx[10] = {0};
    uint8_t rxLen  = sizeof(rx);

    uint8_t st = mfrc522_transceive(cmd, 2, rx, &rxLen, 0);
    if (st)        return st;
    if (rxLen < 5) return 3;                  // expect UID[4] + BCC

    uid[0]=rx[0]; uid[1]=rx[1]; uid[2]=rx[2]; uid[3]=rx[3];
    uint8_t bcc = rx[4];
    if ( (uid[0]^uid[1]^uid[2]^uid[3]^bcc) != 0 ) return 5; // BCC fail
    return 0;
}


static uint8_t picc_select_cl1(const uint8_t uid[4], uint8_t *sak_out)
{
    uint8_t buf[9];   // SEL(1) + NVB(1) + UID(4) + BCC(1) + CRC(2)
    uint8_t crc[2];
    uint8_t rx[10];
    uint8_t rxLen = sizeof(rx);

  
    buf[0] = PICC_SEL_CL1; // 0x93
    buf[1] = 0x70;
    // copy UID[0..3]
    for (int i = 0; i < 4; ++i) buf[2 + i] = uid[i];
    
    buf[6] = uid[0] ^ uid[1] ^ uid[2] ^ uid[3];


    mfrc522_crc(buf, 7, crc);
    buf[7] = crc[0];
    buf[8] = crc[1];

    // send frame and expect 1 byte SAK
    uint8_t st = mfrc522_transceive(buf, 9, rx, &rxLen, 0);
    if (st) return st;
    if (rxLen < 1) return 2;

    *sak_out = rx[0];
    return 0;
}

/* ---- Authorized tag (your UID) ---- */
static const uint8_t AUTH_UID[4] = {0x4B, 0xA8, 0xB1, 0x01};

/* tiny helper */
static int uid_equal(const uint8_t a[4], const uint8_t b[4])
{
    return (a[0]==b[0]) && (a[1]==b[1]) && (a[2]==b[2]) && (a[3]==b[3]);
}

void leds_init(void)
{
    // ensure clocks are on for A, B, C (needed for PB0, PC0, PA10)
    RCC->AHB1ENR |= (1U<<0) | (1U<<1) | (1U<<2);

    // PB0 (green)
    LED_GREEN_PORT->MODER &= ~(3U<<(LED_GREEN_PIN*2));
    LED_GREEN_PORT->MODER |=  (1U<<(LED_GREEN_PIN*2));

    // PC0 (red)
    LED_RED_PORT->MODER &= ~(3U<<(LED_RED_PIN*2));
    LED_RED_PORT->MODER |=  (1U<<(LED_RED_PIN*2));

    // PA10 (yellow)
    LED_YELLOW_PORT->MODER &= ~(3U<<(LED_YELLOW_PIN*2));
    LED_YELLOW_PORT->MODER |=  (1U<<(LED_YELLOW_PIN*2));

    // push-pull, no pulls
    LED_GREEN_PORT->OTYPER  &= ~PIN_MASK(LED_GREEN_PIN);
    LED_RED_PORT->OTYPER    &= ~PIN_MASK(LED_RED_PIN);
    LED_YELLOW_PORT->OTYPER &= ~PIN_MASK(LED_YELLOW_PIN);

    LED_GREEN_PORT->PUPDR  &= ~(3U<<(LED_GREEN_PIN*2));
    LED_RED_PORT->PUPDR    &= ~(3U<<(LED_RED_PIN*2));
    LED_YELLOW_PORT->PUPDR &= ~(3U<<(LED_YELLOW_PIN*2));

        // PA8 (blue)  <-- D7
    LED_BLUE_PORT->MODER &= ~(3U << (LED_BLUE_PIN * 2));
    LED_BLUE_PORT->MODER |=  (1U << (LED_BLUE_PIN * 2));   // output
    LED_BLUE_PORT->OTYPER  &= ~PIN_MASK(LED_BLUE_PIN);     // push-pull
    LED_BLUE_PORT->PUPDR   &= ~(3U << (LED_BLUE_PIN * 2)); // no pulls
    LED_OFF(LED_BLUE_PORT, LED_BLUE_PIN);



    // all OFF initially
    LED_OFF(LED_GREEN_PORT,  LED_GREEN_PIN);
    LED_OFF(LED_RED_PORT,    LED_RED_PIN);
    LED_OFF(LED_YELLOW_PORT, LED_YELLOW_PIN);
}

// 4-digit PIN required after a valid RFID tag
static const char EXPECTED_PIN[4] = { '2','5','8','0' };  // 

static uint8_t check_pin_blocking(uint32_t timeout_ms)
{
    char entered[4] = {0,0,0,0};
    uint8_t n = keypad_read_code(entered, 4, timeout_ms);  // waits up to timeout_ms
    if (n != 4) return 0;  // timed out or incomplete
    return (entered[0]==EXPECTED_PIN[0] &&
            entered[1]==EXPECTED_PIN[1] &&
            entered[2]==EXPECTED_PIN[2] &&
            entered[3]==EXPECTED_PIN[3]);
}



int main(void)
{
    // --- Initialization ---
    enable_gpio_spi1_clocks();
    config_spi1_pins();
    spi1_init_mode0();

    mfrc522_hw_reset();
    mfrc522_soft_reset();
    mfrc522_init_14443a();   // ISO14443A setup + RF ON
    leds_init();
    keypad_init();

    // --- Verify communication ---
    volatile uint8_t g_ver = mfrc522_read(VersionReg);
    volatile uint8_t txc   = mfrc522_read(TxControlReg);
    (void)g_ver; (void)txc;

    // --- Variables ---
    uint8_t atqa[2] = {0,0};
    uint8_t uid[4]  = {0};
    uint8_t sak     = 0;
    uint8_t card_present = 0, last_present = 0;
    uint8_t uid_ok = 0, pin_checked = 0, pin_ok = 0;
    static char uid_str[3*4] = {0};

    // Idle LEDs
    LED_ON(LED_BLUE_PORT, LED_BLUE_PIN);
    LED_OFF(LED_YELLOW_PORT, LED_YELLOW_PIN);
    LED_OFF(LED_GREEN_PORT, LED_GREEN_PIN);
    LED_OFF(LED_RED_PORT, LED_RED_PIN);

    // --- Main loop ---
    for (;;)
    {
        uint8_t st_req = picc_request_a(atqa);
        card_present = (st_req == 0) && (atqa[0] | atqa[1]);

        if (card_present && !last_present)
        {
            // Card just arrived
            LED_OFF(LED_BLUE_PORT, LED_BLUE_PIN);
            LED_ON(LED_YELLOW_PORT, LED_YELLOW_PIN); // Yellow ON for any card

            // Read UID
            uint8_t st = picc_anticoll_cl1(uid);
            uid_to_hex(uid, uid_str);
            (void)picc_select_cl1(uid, &sak);
            (void)st;

            // Check UID match
            uid_ok = uid_equal(uid, AUTH_UID) ? 1 : 0;
            pin_checked = 0;
            pin_ok = 0;

            if (!uid_ok)
            {
                // Wrong card: Red ON, Yellow stays ON
                LED_ON(LED_RED_PORT,    LED_RED_PIN);
                LED_OFF(LED_GREEN_PORT, LED_GREEN_PIN);
            }
            else
            {
                // Correct UID: Stay Yellow (no green yet)
                LED_OFF(LED_RED_PORT,   LED_RED_PIN);
                LED_OFF(LED_GREEN_PORT, LED_GREEN_PIN);
            }
        }

        // Ask for PIN if correct UID and not checked yet
        if (card_present && uid_ok && !pin_checked)
        {
            // Yellow stays ON
            pin_ok = check_pin_blocking(10000); // wait up to 10s for PIN
            pin_checked = 1;

            if (pin_ok)
            {
                // PIN correct → Door open → Green ON (keep Yellow ON)
                LED_ON(LED_GREEN_PORT,  LED_GREEN_PIN);
                LED_OFF(LED_RED_PORT,   LED_RED_PIN);
            }
            else
            {
                // Wrong PIN → Red ON (keep Yellow ON)
                LED_OFF(LED_GREEN_PORT, LED_GREEN_PIN);
                LED_ON(LED_RED_PORT,    LED_RED_PIN);
            }
        }

        // No card present → reset
        if (!card_present)
        {
            uid_ok = 0;
            pin_checked = 0;
            pin_ok = 0;

            LED_ON(LED_BLUE_PORT,   LED_BLUE_PIN);   // idle
            LED_OFF(LED_YELLOW_PORT,LED_YELLOW_PIN);
            LED_OFF(LED_RED_PORT,   LED_RED_PIN);
            LED_OFF(LED_GREEN_PORT, LED_GREEN_PIN);
        }

        last_present = card_present;
        delay_ms(250);
    }
}

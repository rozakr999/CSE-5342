#include "stm32f4xx.h"
#include "lcd_i2c.h"
#include <string.h>

#define LCD_ADDR (0x25 << 1)  // 7-bit address shifted

void delayMs(int n) {
		int i;
		for(; n > 0; n--)
				for (i = 0; i < 1067; i++);
}

// I2C1 on PB8 (SCL) and PB9 (SDA)
void I2C1_init(void)
{
    // Enable GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Set PB8, PB9 to AF4 (I2C1)
    GPIOB->MODER &= ~((3 << (8*2)) | (3 << (9*2)));
    GPIOB->MODER |=  ((2 << (8*2)) | (2 << (9*2))); // AF mode
	
		// Open-drain for I2C
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);

    GPIOB->AFR[1] |= (4 << ((8-8)*4)) | (4 << ((9-8)*4)); // AF4

    // Pull-up
    GPIOB->PUPDR |= (1 << (8*2)) | (1 << (9*2));

    // Enable I2C1 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Reset I2C1
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    // Set frequency (16 MHz APB1 default)
    I2C1->CR2 = 16;

    // Standard mode 100kHz
    I2C1->CCR = 80;      // Approx 100 kHz
    I2C1->TRISE = 17;

    // Enable I2C
    I2C1->CR1 |= I2C_CR1_PE;
}

void i2c_write(uint8_t data)
{
    while (I2C1->SR2 & I2C_SR2_BUSY);

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = LCD_ADDR;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;

    while (!(I2C1->SR1 & I2C_SR1_BTF));

    I2C1->CR1 |= I2C_CR1_STOP;
}

void lcd_write4(uint8_t nibble, uint8_t mode)
{
    uint8_t data = nibble | mode | 0x08; // backlight on

    i2c_write(data | 0x04);  // enable pulse
    i2c_write(data);
}

void lcd_send_cmd(uint8_t cmd)
{
    lcd_write4(cmd & 0xF0, 0);
    lcd_write4((cmd << 4) & 0xF0, 0);
}

void lcd_send_data(uint8_t data)
{
    lcd_write4(data & 0xF0, 1);
    lcd_write4((data << 4) & 0xF0, 1);
}

void lcd_init(void)
{
    // Basic 4-bit init sequence
    lcd_write4(0x30, 0);
    delayMs(5);
    lcd_write4(0x30, 0);
    delayMs(1);
    lcd_write4(0x20, 0);

    lcd_send_cmd(0x28); // 4-bit, 2 line
    lcd_send_cmd(0x0C); // Display on, cursor off
    lcd_send_cmd(0x01); // Clear
    delayMs(2);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    delayMs(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = col + (row ? 0x40 : 0x00);
    lcd_send_cmd(0x80 | addr);
}

void lcd_print(const char *str)
{
    while (*str)
        lcd_send_data(*str++);
}

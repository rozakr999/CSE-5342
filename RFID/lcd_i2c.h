#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>

void I2C1_init(void);
void i2c_write(uint8_t data);
void lcd_write4(uint8_t nibble, uint8_t mode);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char *str);

#endif

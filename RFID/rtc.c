#include <stdio.h>
#include <string.h>
#include "uart.h"

// Forward declarations
void uart_init(void);
void uart_write_string(const char *str);
char uart_read_char(void);

void delayMs(int n) {
		int i;
		for(; n > 0; n--)
				for (i = 0; i < 1067; i++);
}

void initHw(void)
{
		// Enable clocks
		RCC->AHB1ENR |= 1; // clock A
	
		// Configure LED pins
		GPIOA->MODER |= (1 << (5 * 2)); // set PA5 as output
}

void initRTC(void)
{
    // Enable power interface clock
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_DBP;  // Allow access to backup domain
    while (!(PWR->CR & PWR_CR_DBP));  // Wait for access to enable
	
		// Uncomment this only if changing RTC source or doing a clean reinit
		//RCC->BDCR |= RCC_BDCR_BDRST;
    //RCC->BDCR &= ~RCC_BDCR_BDRST; 

    // Enable LSE (32.768 kHz crystal)
    RCC->BDCR |= RCC_BDCR_LSEON;
    while (!(RCC->BDCR & RCC_BDCR_LSERDY)); // Wait until LSE is ready

    // Select LSE as RTC clock and enable RTC
    RCC->BDCR &= ~RCC_BDCR_RTCSEL;
    RCC->BDCR |= RCC_BDCR_RTCSEL_0; // 01: LSE selected
    RCC->BDCR |= RCC_BDCR_RTCEN;    // Enable RTC
	
		// Wait for RTC registers to synchronize
    RTC->ISR &= ~RTC_ISR_RSF;            // Clear RSF
    while (!(RTC->ISR & RTC_ISR_RSF));   // Wait for registers to sync
	
		// Enter RTC init mode to set prescalers
		RTC->WPR = 0xCA;
		RTC->WPR = 0x53;
		RTC->ISR |= RTC_ISR_INIT;
		while (!(RTC->ISR & RTC_ISR_INITF));

		// ck_spre = 32,768 / ((127+1)*(255+1)) = 1 Hz
		RTC->PRER = (127 << 16) | 255;
		
		// Set 24-hour format
    RTC->CR &= ~RTC_CR_FMT;

		RTC->ISR &= ~RTC_ISR_INIT;
		RTC->WPR = 0xFF;
}

uint8_t bcd2dec(uint8_t val) { return ((val >> 4)*10) + (val & 0x0F); }

void getTime(char *buffer)
{
    uint32_t tr = RTC->TR;

    uint8_t hours = bcd2dec((tr >> 16) & 0x3F);
    uint8_t minutes = bcd2dec((tr >> 8) & 0x7F);
    uint8_t seconds = bcd2dec(tr & 0x7F);

    sprintf(buffer, "%02u:%02u:%02u\r\n", hours, minutes, seconds);
}

uint32_t dec2bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

void setTime(uint8_t hours, uint8_t minutes, uint8_t seconds) {
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    RTC->ISR |= RTC_ISR_INIT;
    while (!(RTC->ISR & RTC_ISR_INITF));

    RTC->TR = (dec2bcd(hours) << 16) |
              (dec2bcd(minutes) << 8) |
              (dec2bcd(seconds) << 0);

    RTC->ISR &= ~RTC_ISR_INIT;
    RTC->WPR = 0xFF;
}

void resetData(USER_DATA* data)
{
		int i;
		for (i = 0; i < MAX_CHARS; i++)
				data->buffer[i] = '\0';
}

int main(void)
{
		initHw();
    initUsart2();
		initRTC();
		setTime(0, 20, 0);
		putsUart("time is set\r\n");
	
		USER_DATA data;

    while (1)
    {
				resetData(&data);
				getData(&data);

				if (strcmp(data.buffer, "y") == 0) {
						putsUart("Entered at: ");
						char msg[32];
						getTime(msg);
						putsUart(msg);
				} 
				if (strcmp(data.buffer, "n") == 0) {
						putsUart("Card failed at: ");
						char msg[32];
						getTime(msg);
						putsUart(msg);
				} 
    }
}

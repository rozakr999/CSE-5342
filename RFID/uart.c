#include "uart.h"
#include "stm32f446xx.h"

#define USART_BAUDRATE 115200
#define SYSCLK 16000000UL

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t sysclk, uint32_t baudrate);

void initUsart2(void)
{
    // Enable clocks for USART2
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure PA2 (TX) and PA3 (RX)
    GPIOA->AFR[0] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
		GPIOA->AFR[0] |= (7U << (2 * 4)) | (7U << (3 * 4)); 			// AF7
		GPIOA->MODER &= ~((3U << (2 * 2)) | (3U << (3 * 2)));  		// Clear mode bits
		GPIOA->MODER |= (2U << (2 * 2)) | (2U << (3 * 2));     		// Set to AF mode
		GPIOA->OSPEEDR |= (3U << (9 * 2)) | (3U << (10 * 2)); // High speed

    // Configure USART2
    USART2->CR1 = 0;                   // Disable USART before config
    uart_set_baudrate(USART2, SYSCLK, USART_BAUDRATE);

    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;  // Enable TX and RX
    USART2->CR1 |= USART_CR1_UE;                 // Enable USART
}

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t sysclk, uint32_t baudrate)
{
    // USARTDIV = Fck / (16 * baud)
    uint32_t usartdiv = sysclk / baudrate;
    USARTx->BRR = usartdiv;
}

void putcUart(char c)
{
    while (!(USART2->SR & USART_SR_TXE)); // Wait until TX buffer empty
    USART2->DR = c;
}

void putsUart(char* str)
{
		uint8_t i = 0;
    while (str[i] != '\0')
        putcUart(str[i++]);
}

char getcUart(void)
{
    while (!(USART2->SR & USART_SR_RXNE)); // Wait until RX buffer full
    return USART2->DR & 0xFF;
}

void getData(USER_DATA* data)
{
	int count = 0;
	int exit = 0;
	while (!exit)
	{
		char c = getcUart();
		if ((c == 8 || c == 127) && count > 0) {
				count--;
				putcUart('\b');
				putcUart(' ');
				putcUart('\b');
		}
		else if (c == 13) {
				data->buffer[count] = 0;
				exit = 1;
		}
		else if (c >= 32 && count!= MAX_CHARS) {
				data->buffer[count] = c;
				count++;
		}
		else if (count == MAX_CHARS) {
				data->buffer[count] = 0;
				exit = 1;
		}
		putcUart(c);
	}
	putsUart("\n");
	return;
}
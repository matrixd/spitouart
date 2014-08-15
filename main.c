/*
 * main.c
 */
#include "stm32f4xx.h"
volatile uint16_t delay_time = 0;
typedef struct {
	uint8_t read;
	uint8_t write;
	uint8_t buf[1024];
} BUF;
BUF to_uart;
BUF to_spi;
void ledInit(void){
	RCC->AHB1ENR = RCC_AHB1ENR_GPIODEN;
	GPIOD->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR12);
	GPIOD->MODER &= ~(GPIO_MODER_MODER12);
	GPIOD->MODER |= GPIO_MODER_MODER12_0;
}
void usartInit(void){
	//RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
	//RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	//RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	//GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR2 | GPIO_MODER_MODER3);
	GPIOA->OSPEEDR = GPIO_OSPEEDER_OSPEEDR10 | GPIO_MODER_MODER9;
	//GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3);
	GPIOA->PUPDR = GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0;
	//GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
	GPIOA->MODER = GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
	GPIOA->AFR[1] = 0x770;
	//USART1->BRR = 0x2d9; // 45.5625
	USART1->BRR = 0x9c4;
	//USART2->CR2 &= ~(USART_CR2_CLKEN);//!!!
	//USART2->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_UE
	//		| USART_CR1_TCIE | USART_CR1_IDLEIE | USART_CR1_TE | USART_CR1_RE);
	USART1->CR1 &= ~( USART_CR1_TE | USART_CR1_UE);
	USART1->CR1 |= USART_CR1_UE | USART_CR1_TE;//USART_CR1_RXNEIE |
}
void spiInit(void){
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5
			         | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1
				         | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	SPI1->CR1 &= ~(SPI_CR1_SSM | SPI_CR1_BR | SPI_CR1_MSTR);
	SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_MSTR;
	SPI1->CR2 &= ~(SPI_CR2_SSOE | SPI_CR2_RXNEIE | SPI_CR2_TXEIE);
	SPI1->CR2 |= SPI_CR2_SSOE;
	SPI1-> CR2 |= SPI_CR2_RXNEIE;
}
void delay(uint16_t time){
	delay_time = time;
	while(delay_time != 0);
}
void blink(void){
	GPIOD->ODR &= ~GPIO_ODR_ODR_12;
	delay(100);
	GPIOD->ODR |= GPIO_ODR_ODR_12;
	delay(100);
}
int main(void){
	SysTick_Config(SystemCoreClock / 1000);
	to_uart.write = 0;
	to_uart.read = 0;
	to_spi.read = 0;
	to_spi.write = 0;
	ledInit();
	usartInit();
	//spiInit();
	uint8_t cfg = 1;
	GPIOD->ODR |= GPIO_ODR_ODR_12;
	//GPIOD->ODR &= ~GPIO_ODR_ODR_12;
	while(1){
		while(cfg == 1){
			//GPIOD->BSRRL |= GPIO_BSRR_BR_12;
			while(!(USART1->SR & USART_SR_TC));
			//USART2->DR = ((uint16_t)0x10  & (uint16_t)0x01FF);
			USART1->DR = 0x35;
			blink();
		}
		while(cfg == 2){
			//GPIOD->BSRRL &= ~GPIO_BSRR_BS_12;
			USART1->DR = 0x35;
			while(USART1->SR | USART_SR_TC);
			GPIOD->BSRRL |= GPIO_BSRR_BS_12;
		}/*
		if(to_uart.write > 0){
			while(to_uart.write > to_uart.read){
				USART1->DR = to_uart.buf[to_uart.read];
				to_uart.read++;
				while(USART1->SR | USART_SR_TC);
			}
			to_uart.write = 0;
			to_uart.read = 0;
		}
		if(to_spi.write > 0){
			while(to_spi.write > to_spi.read){
				SPI1->DR = to_spi.buf[to_spi.read];
				to_spi.read++;
				while(SPI1->SR | SPI_SR_TXE);
			}
			to_spi.write = 0;
			to_spi.read = 0;
		}*/
	}
	return 0;
}

/*void SPI1_IRQHandler(void){
	if(SPI1->SR & SPI_SR_RXNE){
		to_uart.buf[to_uart.write];
		to_uart.write++;
	}
}
void USART2_IRQHandler(void){
	if(USART1->SR & USART_SR_RXNE){
		to_spi.buf[to_spi.write];
		to_uart.write++;
	}
}
*/
void SysTick_Handler(void){
	if(delay_time != 0){
		delay_time--;
	}
}

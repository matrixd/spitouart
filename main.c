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
	RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR12);
	GPIOD->MODER &= ~(GPIO_MODER_MODER12);
	GPIOD->MODER |= GPIO_MODER_MODER12_0;
}
void usartInit(void){
	RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6 | GPIO_MODER_MODER7);
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_MODER_MODER7;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;
	GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOB->AFR[0] &= ~0x77000000;
	GPIOB->AFR[0] |= 0x77000000;
	USART1->CR1 = USART_CR1_UE;
	USART1->BRR = 0x2d9; // 45.5625
	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE;
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 2);
}
void spiInit(void){
	RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR &= RCC_APB2ENR_SPI1EN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5
			| GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5
			| GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5
			| GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0
			| GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;
	GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5
			         | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1
				         | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOA->AFR[0] &= ~0x55550000;
	GPIOA->AFR[0] |= 0x55550000;
	SPI1->CR1 = SPI_CR1_BR;
	SPI1-> CR2 = 0x10 | SPI_CR2_RXNEIE;
	SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;
	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_SetPriority(SPI1_IRQn, 5);
}
void delay(uint16_t time){
	delay_time = time;
	while(delay_time != 0);
}
void blink(void){
	GPIOD->ODR &= ~GPIO_ODR_ODR_12;
	GPIOD->ODR |= GPIO_ODR_ODR_12;
	delay(100);
	GPIOD->ODR &= ~GPIO_ODR_ODR_12;
	delay(100);
}
int main(void){
	SysTick_Config(SystemCoreClock / 1000);
	to_uart.write = 0;
	to_uart.read = 0;
	to_spi.read = 0;
	to_spi.write = 0;
	ledInit();
	spiInit();
	usartInit();
	uint8_t cfg = 0;
	while(1){
		//test for usart
		while(cfg == 1){
			while(!(USART1->SR & USART_SR_TC));
			USART1->DR = 0x35;
			blink();
		}
		//test for spi
		if(cfg == 3){
			to_spi.write = 1;
			to_spi.buf[0] = 0x07;
			blink();
		}
		if(to_uart.write > 0){
			while(to_uart.write > to_uart.read){
				USART1->DR = to_uart.buf[to_uart.read]+0x30;
				to_uart.read++;
				while(!(USART1->SR & USART_SR_TC));
			}
			to_uart.write = 0;
			to_uart.read = 0;
		}
		if(to_spi.write > 0){
			while(to_spi.write > to_spi.read){
				SPI1->DR = to_spi.buf[to_spi.read]-0x30;
				to_spi.read++;
				while(!(SPI1->SR & SPI_SR_TXE));
			}
			to_spi.write = 0;
			to_spi.read = 0;
		}
	}
	return 0;
}

void SPI1_IRQHandler(void){
	if(SPI1->SR & SPI_SR_RXNE){
		to_uart.buf[to_uart.write] = SPI1->DR;
		to_uart.write++;
	}
}
void USART1_IRQHandler(void){
	if(USART1->SR & USART_SR_RXNE){
		to_spi.buf[to_spi.write] = USART1->DR;
		to_spi.write++;
	}
}

void SysTick_Handler(void){
	if(delay_time != 0){
		delay_time--;
	}
}

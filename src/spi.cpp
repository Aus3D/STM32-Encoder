#include <stdint.h>
#include "stm32f0xx.h"

void initSPI()
{
	//Configure clocks
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Turn on the clock for the SPI interface
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Turn on PORT A

	// Configure the pins
	// SPI1 MOSI is on pin 13 (PA7) AF0.  This is labelled A6 on the Nucleo board
	GPIOA->MODER |= GPIO_MODER_MODER7_1;
	GPIOA->MODER &= ~GPIO_MODER_MODER7_0;
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFRL7));

	SPI1->CR1 = SPI1->CR2 = 0; 	// Start with zero in the control registers.
	SPI1->CR1 |= SPI_CR1_CPHA;	// set CPHA to ensure MOSI is low when idle
	SPI1->CR1 |= SPI_CR1_MSTR;	// select master mode
	SPI1->CR1 |= (3 << 3);// select divider of 16.  48MHz/16 = 3MHz.  3 bits per WS2812 bit so: 1 Million WSbits/second : Within range of  1.53MHz to 540kHz
	SPI1->CR1 |= (SPI_CR1_SSI | SPI_CR1_SSM);// select software slave management and drive SSI high (not relevant in TI mode and not output to pins)
	SPI1->CR2 |= (7 << 8);// select 8 bit data transfer size
	SPI1->CR2 |= SPI_CR2_TXDMAEN;// enable transmit DMA
	SPI1->CR1 |= SPI_CR1_SPE;// enable SPI1

	// DMA configuration: SPI1 TX is on DMA channel 3
	RCC->AHBENR |= RCC_AHBENR_DMA1EN; // enable clocks for DMA controller
	DMA1_Channel3->CPAR = (uint32_t)&(SPI1->DR);
	DMA1_Channel3->CMAR = 0;// Don't know what to send yet
	DMA1_Channel3->CNDTR = 0;// No bytes yet

	DMA1_Channel3->CCR |= DMA_CCR_PL_1 | DMA_CCR_PL_0;	//very high priority
	DMA1_Channel3->CCR |= DMA_CCR_MINC;	//memory increment mode
	DMA1_Channel3->CCR |= DMA_CCR_DIR;	//read from memory
	DMA1_Channel3->CCR |= DMA_CCR_EN;	//enable DMA

}
void writeSPI(uint8_t *DMABuffer,unsigned Count)
{	
	DMA1_Channel3->CCR &= ~DMA_CCR_EN; // disable DMA
	DMA1_Channel3->CNDTR = Count; // Tell the DMA controller how many bytes are to be sent
	DMA1_Channel3->CMAR = (uint32_t)DMABuffer;  // Set DMA source
	DMA1->IFCR = 0x0f; // clear any pending interrupts
	DMA1_Channel3->CCR |= DMA_CCR_EN; // re-enable DMA
}


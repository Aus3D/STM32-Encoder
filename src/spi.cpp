#include <stdint.h>
#include "stm32f0xx.h"
//#include "stm32f030.h"

//#define OLDMETHOD

#ifdef OLDMETHOD
	#include "stm32f030.h"
#endif

void initSPI()
{

	#ifdef OLDMETHOD
    	RCC_APB2ENR |= BIT12;
		RCC_AHBENR |= BIT17;
	#else
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Turn on the clock for the SPI interface
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Turn on PORT A
	#endif

	// Configure the pins
	// SPI1 MOSI is on pin 13 (PA7) AF0.  This is labelled A6 on the Nucleo board
	#ifdef OLDMETHOD
		GPIOA_MODER |= BIT15;
		GPIOA_MODER &= ~BIT14;
		GPIOA_AFRL &= ~(BIT31+BIT30+BIT29+BIT28);
	#else
	    GPIOA->MODER |= GPIO_MODER_MODER7_1;
	    GPIOA->MODER &= ~GPIO_MODER_MODER7_0;
	    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFRL7));
	#endif

	#ifdef OLDMETHOD
		SPI1_CR1 = SPI1_CR2 = 0; // Start with zero in the control registers.
		SPI1_CR1 |= BIT0; // set CPHA to ensure MOSI is low when idle
		SPI1_CR1 |= BIT2; // select master mode
		SPI1_CR1 |= (3 << 3); // select divider of 16.  48MHz/16 = 3MHz.  3 bits per WS2812 bit so: 1 Million WSbits/second : Within range of  1.53MHz to 540kHz
		SPI1_CR1 |= BIT8+BIT9; // select software slave management and drive SSI high (not relevant in TI mode and not output to pins)
		SPI1_CR2 |= (7 << 8); // select 8 bit data transfer size
		SPI1_CR2 |= BIT1; // enable transmit DMA
		SPI1_CR1 |= BIT6; // enable SPI1
	#else
	    SPI1->CR1 = SPI1->CR2 = 0;
	    SPI1->CR1 |= SPI_CR1_CPHA;
	    SPI1->CR1 |= SPI_CR1_MSTR;
	    SPI1->CR1 |= (3 << 3);
	    SPI1->CR1 |= (SPI_CR1_SSI | SPI_CR1_SSM);
	    SPI1->CR2 |= (7 << 8);
	    SPI1->CR2 |= SPI_CR2_TXDMAEN;
	    SPI1->CR1 |= SPI_CR1_SPE;
	#endif

	#ifdef OLDMETHOD
	    RCC_AHBENR |= BIT0; // enable clocks for DMA controller
	    DMA_CPAR3 = (unsigned long)(&(SPI1_DR));
	    DMA_CMAR3 = 0;  // Don't know what to send yet
	    DMA_CNDTR3 = 0; // No bytes yet
	    DMA_CCR3 = BIT13+BIT12+BIT7+BIT4; // Select high priority, 8 bits, Memory increment mode (only), 	Read from memory, enable
	#else
		// DMA configuration: SPI1 TX is on DMA channel 3
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
		DMA1_Channel3->CPAR = (uint32_t)&(SPI1->DR);
		DMA1_Channel3->CMAR = 0;
		DMA1_Channel3->CNDTR = 0;

		DMA1_Channel3->CCR |= DMA_CCR_PL_1 | DMA_CCR_PL_0;	//very high priority
		DMA1_Channel3->CCR |= DMA_CCR_MINC;	//memory increment mode
		DMA1_Channel3->CCR |= DMA_CCR_DIR;	//read from memory
		DMA1_Channel3->CCR |= DMA_CCR_EN;	//enable DMA
	#endif

}
void writeSPI(uint8_t *DMABuffer,unsigned Count)
{	
	#ifdef OLDMETHOD
		DMA_CCR3 &= ~BIT0; // disable DMA
		DMA_CNDTR3 = Count; // Tell the DMA controller how many bytes are to be sent
		DMA_CMAR3 = (unsigned long)DMABuffer; // Set DMA source
		DMA_IFCR = 0x0f; // clear any pending interrupts
		DMA_CCR3 |= BIT0; // re-enable DMA
	#else
		DMA1_Channel3->CCR &= ~DMA_CCR_EN;
		DMA1_Channel3->CNDTR = Count;
		DMA1_Channel3->CMAR = (uint32_t)DMABuffer;
		DMA1->IFCR = 0x0f;
		DMA1_Channel3->CCR |= DMA_CCR_EN;
	#endif
}


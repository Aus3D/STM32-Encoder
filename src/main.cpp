/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "main.h"
#include "stm32f0xx.h"
#include <stdint.h>
#include "ws2812.h"
#include "encoder.h"
#include <stdio.h>
#include <string.h>
#include "pins.h"

int ledBrightness[] = {20,20};
int ledMode[]       = {0,0};
int ledRate[]       = {0,0};
int ledSleep[]      = {0,0};

uint8_t ledRGB[] = {255,0,0};
uint8_t ledHSV[] = {255,255,255};

#define I2C_TIMING      0x00A51314
#define I2C_ADDRESS        (0x30)
#define I2C_BYTE_TO_SEND (0x01)
#define MAX_DATA_BYTES 8

uint8_t m_TransmissionRuning = 0;

uint8_t rxIndex = 0;
bool 	rxComplete = false;
uint8_t txIndex = 0;
uint8_t rxBuffer[MAX_DATA_BYTES] = {0};
uint8_t txBuffer[MAX_DATA_BYTES] = {0};

Ws2812 pixel = Ws2812();
AS5600Encoder encoder = AS5600Encoder();

I2C_HandleTypeDef hi2c1;

int main(void)
{
	setup();

	for(;;) {
		encoder.update();

		updateLed();

		if(false) {	//rxComplete
			//copy received data into new array, in case RX data is overridden in I2C interrupt
			uint8_t temp[MAX_DATA_BYTES];
			memcpy(rxBuffer,temp,MAX_DATA_BYTES);

			blinkLeds(1,CRGB::Orange);

			for(int i = 0; i < 1; i++) {
				for(int j = 0; j < 8; j++) {
					if(CHECK_BIT(temp[i],7-j)) {
						blinkLeds(1,CRGB::White);
					} else {
						blinkLeds(1,CRGB::Blue);
					}
				}
			}

			blinkLeds(1,CRGB::Orange);
		}
	}
}

void setup() {
	SystemClock_Config();

	HAL_Init();

	HAL_Delay(10);

	Configure_GPIO_I2C1();
	Configure_I2C1_Slave();
	Configure_GPIO_General();

	pixel.setup();

	HAL_Delay(10);

	for(uint8_t i = 0; i < 100; i++) {
		TEST_PORT->BSRR = TEST_PIN;
		HAL_Delay(10);
		TEST_PORT->BRR = TEST_PIN;
		HAL_Delay(10);
	}

	if(encoder.init()) {
		blinkLeds(1,CRGB::Green);
	} else {
		blinkLeds(1,CRGB::Red);
	}

	blinkLeds(1,CRGB::Green);
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the I2C1 pins on GPIO PF0 PF1
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_I2C1(void)
{
  /* Enable the peripheral clock of GPIOF */
  RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

  /* (1) open drain for I2C signals */
  /* (2) AF1 for I2C signals */
  /* (3) Select AF mode (10) on PF0 and PF1 */
  GPIOF->OTYPER |= GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1; /* (1) */
  GPIOF->AFR[0] = (GPIOF->AFR[0] & ~(GPIO_AFRL_AFRL0 | GPIO_AFRL_AFRL1))  | (1 << ( 0 * 4 )) | (1 << (1 * 4)); /* (2) */
  GPIOF->MODER = (GPIOF->MODER & ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1)) | (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1); /* (3) */

}

/**
  * @brief  This function configures I2C1, slave.
  * @param  None
  * @retval None
  */
__INLINE void Configure_I2C1_Slave(void)
{
  /* Configure RCC for I2C1 */
  /* (1) Enable the peripheral clock I2C1 */
  /* (2) Use SysClk for I2C CLK */
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; /* (1) */
  RCC->CFGR3 |= RCC_CFGR3_I2C1SW_SYSCLK; /* (2) */

  /* Configure I2C1, slave */
  /* (3) Timing register value is computed with the AN4235 xls file,
         fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns,
         fall time = 40ns */
  /* (4) Periph enable, address match interrupt enable */
  /* (5) 7-bit address = 0x5A */
  /* (6) Enable own address 1 */
  I2C1->TIMINGR = (uint32_t)0x00B00000; /* (3) */
  I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE | I2C_CR1_ADDRIE |I2C_CR1_STOPIE; /* (4) */
  I2C1->OAR1 |= (uint32_t)(I2C_ADDRESS<<1); /* (5) */
  //I2C1->OAR1 |= I2C_OAR1_OA1MODE;
  //I2C1->CR1 |= I2C_CR1_SBC;
  I2C1->OAR1 |= I2C_OAR1_OA1EN; /* (6) */

  /* Configure IT */
  /* (7) Set priority for I2C1_IRQn */
  /* (8) Enable I2C1_IRQn */
  NVIC_SetPriority(I2C1_IRQn, 0); /* (7) */
  NVIC_EnableIRQ(I2C1_IRQn); /* (8) */
}


void Configure_GPIO_General(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEST_PORT, TEST_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA13 */
  GPIO_InitStruct.Pin = TEST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TEST_PORT, &GPIO_InitStruct);

}


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void updateLed() {
	for (int i = 0; i < 1; i++) {
		switch (ledMode[i]) {
		case 0:
			if (encoder.getMagStrength() == MAG_SIG_GOOD) {
				pixel.setPixelColor(i, CRGB::Green);
			} else if (encoder.getMagStrength() == MAG_SIG_MID) {
				pixel.setPixelColor(i, CRGB::Yellow);
			} else if (encoder.getMagStrength() == MAG_SIG_BAD) {
				pixel.setPixelColor(i, CRGB::Red);
			}
			break;
		case 1:
			pixel.setPixelColor(i, CRGB::White);
			break;
		case 2:
			pixel.setPixelColor(i, CRGB::Red);
			break;
		case 3:
			pixel.setPixelColor(i, CRGB::Green);
			break;
		case 4:
			pixel.setPixelColor(i, CRGB::Blue);
			break;
		case 5:
			pixel.setPixelColor(i, ledRGB[0], ledRGB[1], ledRGB[2]);
			break;
		case 6:
			//leds[i].setHSV(ledHSV[0], ledHSV[1], ledHSV[2]);
			break;
		case 7:
			//leds[i].setHSV(encoderCount.val / 10 * ledRate[i], 255, 255);
			break;
		case 8:
			//leds[i].setHSV(millis() / 1000 * ledRate[i], 255, 255);
			if(encoder.getCount() <= 512) {
				pixel.setPixelColor(i, (uint8_t)encoder.getCount()/2, 0, 0);
			} else if (encoder.getCount() <= 1024) {
				pixel.setPixelColor(i, 255 - ((uint8_t)(encoder.getCount()-512)/2), 0, 0);
			} else if (encoder.getCount() <= 1536) {
				pixel.setPixelColor(i, 0, ((uint8_t)(encoder.getCount()-1024)/2), 0);
			} else if (encoder.getCount() <= 2048) {
				pixel.setPixelColor(i, 0, 255 - ((uint8_t)(encoder.getCount()-1536)/2), 0);
			} else if (encoder.getCount() <= 2560) {
				pixel.setPixelColor(i, 0, 0, ((uint8_t)(encoder.getCount()-2048)/2));
			} else if (encoder.getCount() <= 3072) {
				pixel.setPixelColor(i, 0, 0, 255 - ((uint8_t)(encoder.getCount()-2560)/2));
			} else if (encoder.getCount() <= 3584) {
				pixel.setPixelColor(i, ((uint8_t)(encoder.getCount()-3072)/2), ((uint8_t)(encoder.getCount()-3072)/2), ((uint8_t)(encoder.getCount()-3072)/2));
			} else if (encoder.getCount() <= 4096) {
				pixel.setPixelColor(i, 255 - ((uint8_t)(encoder.getCount()-3584)/2), 255 - ((uint8_t)(encoder.getCount()-3584)/2), 255 - ((uint8_t)(encoder.getCount()-3584)/2));
			}
			//uint8_t brightness = encoder.getCount() / 16;
			//pixel.setPixelColor(i,brightness,brightness,brightness);
			break;
		}
		pixel.nscale8(i,ledBrightness[i]);
	}
	pixel.show();
}

void blinkLeds(int times, uint32_t color) {
	blinkLeds(times,(uint8_t)(color >> 16),(uint8_t)(color >> 8),(uint8_t)(color));
}

void blinkLeds(int times, uint8_t red, uint8_t green, uint8_t blue) {
  for(int i = 0; i < times; i++) {
    for(int j = 0; j < pixel.numPixels(); j++) {
    	pixel.setPixelColor(j,red,green,blue);
    	pixel.nscale8(j,ledBrightness[i]);
    }
    pixel.show();
    HAL_Delay(500);
    for(int j = 0; j < pixel.numPixels(); j++) {
     	pixel.setPixelColor(j,0);
     }
    pixel.show();
    HAL_Delay(500);
  }
}

void i2c_receive_callback() {

	//ledMode[0] = rxBuffer[2];


	//copy received data into new array, in case RX data is overridden in I2C interrupt
	uint8_t temp[MAX_DATA_BYTES];
	memcpy(temp,rxBuffer,MAX_DATA_BYTES);

	//ledMode[0] = rxBuffer[2];



	switch(temp[0]) {
	    case I2C_RESET_COUNT:
	      //offset = encoderCount.val;
	      break;
	    case I2C_SET_ADDR:
	      //setI2cAddress(temp[1]);
	      //blinkLeds(1,CRGB::White);
	      //restart();
	      break;
	    case I2C_SET_REPORT_MODE:
	      i2c_response_mode = temp[1];
	      break;
	    case I2C_CLEAR_EEPROM:
	      //eepromClear();
	      break;
	    case I2C_ENC_LED_PAR_MODE:
	      setLedMode(temp[1],temp[2]);
	      break;
	    case I2C_ENC_LED_PAR_BRT:
	      setLedBrightness(temp[1],temp[2]);
	      break;
	    case I2C_ENC_LED_PAR_RGB:
	      //setLedRGB(temp[1],temp[2],temp[3]);
	      break;
	    case I2C_ENC_LED_PAR_HSV:
	      //setLedHSV(temp[1],temp[2],temp[3]);
	      break;
	    case I2C_ENC_LED_PAR_RATE:
	      //setLedRate(temp[1],temp[2]);
	      break;
	  }



}

void i2c_callback() {


	uint32_t I2C_InterruptStatus = I2C1->ISR; // Get interrupt status //

	if((I2C_InterruptStatus & I2C_ISR_ADDR) == I2C_ISR_ADDR) // Check address match //
	{
		I2C1->ICR |= I2C_ICR_ADDRCF; // Clear address match flag //
		//if((I2C1->ISR & I2C_ISR_DIR) == I2C_ISR_DIR) // Check if transfer direction is read (slave transmitter) //
		//{
		//	I2C1->CR1 |= I2C_CR1_TXIE; // Set transmit IT //
		//}
	}
	//else if((I2C_InterruptStatus & I2C_ISR_TXIS) == I2C_ISR_TXIS)
	//{
	//	I2C1->CR1 &=~ I2C_CR1_TXIE; // Disable transmit IT //
	//	I2C1->TXDR = I2C_BYTE_TO_SEND; // Byte to send //
	//}
	else if((I2C_InterruptStatus & I2C_ISR_RXNE) == I2C_ISR_RXNE)
	{
		ledMode[0] = I2C1->RXDR;

		//GPIOC->BSRR = GPIO_BSRR_BS_8; // Lit orange LED //
		//NVIC_DisableIRQ(I2C1_IRQn); // Disable I2C1_IRQn //
	//}
	//else if((I2C_InterruptStatus & I2C_ISR_NACKF) == I2C_ISR_NACKF)	// If NACK event
	//{
	//		// Clear NACK flag
	//		I2C1->ICR |= I2C_ICR_NACKCF;
	//}
//	else if((I2C_InterruptStatus & I2C_ISR_STOPF) == I2C_ISR_STOPF)		// If STOP event
//	{
//			// Clear STOP flag
//			I2C1->ICR |= I2C_ICR_STOPCF;
	} else {
		NVIC_DisableIRQ(I2C1_IRQn); /* Disable I2C1_IRQn */
	}


}

extern "C"
{

	/**
	  * @brief  This function handles I2C1 interrupt request.
	  * @param  None
	  * @retval None
	  */
	void I2C1_IRQHandler(void)
	{
		TEST_PORT->BSRR |= TEST_PIN;		//debug test
		//i2c_callback();

		uint32_t I2C_InterruptStatus = I2C1->ISR; /* Get interrupt status */

		if((I2C_InterruptStatus & I2C_ISR_ADDR) == I2C_ISR_ADDR)
		{
			I2C1->ICR |= I2C_ICR_ADDRCF; /* Address match event */

			rxIndex = 0;
			txIndex = 0;
			rxComplete = false;
		}
		else if((I2C_InterruptStatus & I2C_ISR_RXNE) == I2C_ISR_RXNE)
		{
			/* Read receive register, will clear RXNE flag */
			//if(I2C1->RXDR == I2C_BYTE_TO_SEND)
			//{
				//GPIOC->ODR ^= GPIO_ODR_9; /* toggle green LED */
			rxBuffer[rxIndex] = I2C1->RXDR;
			rxIndex++;
				//ledMode[0] = I2C1->RXDR;
			//}
		}
		else if ((I2C_InterruptStatus & I2C_ISR_STOPF) == I2C_ISR_STOPF)
		{
			if(rxIndex > 0) {
				i2c_receive_callback();
				rxComplete = true;
			}


			I2C1->ICR |= I2C_ICR_STOPCF;
		}

		TEST_PORT->BRR |= TEST_PIN;		//debug test
	}

//	void HAL_I2C_SlaveRXCpltCallback(I2C_HandleTypeDef *I2cH) {
//
//		pixel.setPixelColor(0, CRGB::Blue);
//		pixel.show();
//
//
//		HAL_StatusTypeDef status;
//		uint8_t *reg_addr;
//		uint16_t max_size;
//		uint16_t num_bytes_received = I2cH->XferSize - I2cH->XferCount;
//		uint8_t *rx_bytes_start = I2cH->pBuffPtr - num_bytes_received;
//
//		bool i2c_transmitting = false;
//		bool write = false;
//
//		if(I2cH->Instance == I2C1) {
//			if(I2cH->Instance->ISR & ((uint32_t)0x00000010)) {	//check if address received was read mode
//			} else {
//				//device enters transmit mode
//				uint16_t i2c_tx_length = 4;
//				uint8_t i2c_tx_buffer[i2c_tx_length];
//
//				memcpy(i2c_tx_buffer,&encoder.encoderCount,4);
//
//				while(HAL_I2C_Slave_Transmit_IT(I2cH,(uint8_t*)i2c_tx_buffer,i2c_tx_length) == HAL_BUSY) {	//&I2cHandle, i2c_tx_buffer, i2c_tx_length
//					//keep trying
//				}
//			}
//		}
//	}
}

void setLedMode(uint8_t led, uint8_t mode) {
	ledMode[constrain(led,0,WS2812_NUM)] = mode;
}

void setLedBrightness(uint8_t led, uint8_t brightness) {
	ledBrightness[constrain(led,0,WS2812_NUM)] = brightness;
}

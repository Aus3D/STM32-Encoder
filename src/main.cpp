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

int ledBrightness[] = {20,20};
int ledMode[]       = {0,0};
int ledRate[]       = {0,0};
int ledSleep[]      = {0,0};

uint8_t ledRGB[] = {255,0,0};
uint8_t ledHSV[] = {255,255,255};

#define I2C_TIMING      0x00A51314
#define I2C_ADDRESS        0x30F



/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;


/* Size of Transmission buffer */
#define TXBUFFERSIZE	32
/* Size of Reception buffer */
#define RXBUFFERSIZE    TXBUFFERSIZE

uint8_t aRxBuffer[RXBUFFERSIZE];

Ws2812 pixel = Ws2812();
//SoftwareWire encWire = SoftwareWire(ENC_I2C_SDA, ENC_I2C_PORT, ENC_I2C_SCL, ENC_I2C_PORT);
AS5600Encoder encoder = AS5600Encoder();

int main(void)
{
	setup();

	for(;;) {
		encoder.update();
		updateLed();

		/*
		blinkLeds(1,CRGB::Orange);
		for(int i = 0; i < 8; i++) {
			if((encoder.statusByte >> 8-i) & 0x01) {
				blinkLeds(1,CRGB::White);
			} else {
				blinkLeds(1,CRGB::Blue);
			}
		}
		blinkLeds(1,CRGB::Orange);
		*/

		if(HAL_GetTick() > 5000) {
			ledMode[0] = 8;
		}
	}

}

void setup() {
	SystemClock_Config();
	HAL_Init();

	pixel.setup();

	blinkLeds(1,CRGB::Green);

	if(encoder.init()) {
		blinkLeds(1,CRGB::Green);
	} else {
		blinkLeds(1,CRGB::Red);
	}

	//set up I2C GPIO

	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	GPIO_InitTypeDef initPort;
	initPort.Pin 		= GPIO_PIN_0 | GPIO_PIN_1;
	initPort.Mode 		= GPIO_MODE_AF_OD;
	initPort.Pull 		= GPIO_PULLUP;
	initPort.Speed 		= GPIO_SPEED_FREQ_HIGH;
	initPort.Alternate 	= GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOF, &initPort);

	I2cHandle.Instance 				= I2C1;
	I2cHandle.Init.Timing          	= I2C_TIMING;
	I2cHandle.Init.OwnAddress1     	= I2C_ADDRESS;
	I2cHandle.Init.AddressingMode	= I2C_ADDRESSINGMODE_7BIT;
	I2cHandle.Init.DualAddressMode	= I2C_DUALADDRESS_DISABLE;
	I2cHandle.Init.OwnAddress2		= 0xFF;
	I2cHandle.Init.GeneralCallMode	= I2C_GENERALCALL_DISABLE;
	I2cHandle.Init.NoStretchMode	= I2C_NOSTRETCH_DISABLE;

	if(HAL_I2C_Init(&I2cHandle) != HAL_OK) {
		blinkLeds(1,CRGB::Purple);
	}

	//NVIC_SystemReset();

	blinkLeds(1,CRGB::Green);
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

void HAL_I2C_SlaveRXCpltCallback(I2C_HandleTypeDef *I2cH) {
	HAL_StatusTypeDef status;
	uint8_t *reg_addr;
	uint16_t max_size;
	uint16_t num_bytes_received = I2cH->XferSize - I2cH->XferCount;
	uint8_t *rx_bytes_start = I2cH->pBuffPtr - num_bytes_received;

	bool i2c_transmitting = false;
	bool write = false;

	if(I2cH->Instance == I2C1) {
		if(I2cH->Instance->ISR & ((uint32_t)0x00000010)) {	//check if address received was read mode
		} else {
			//device enters transmit mode
			uint16_t i2c_tx_length = 4;
			uint8_t i2c_tx_buffer[i2c_tx_length];

			memcpy(i2c_tx_buffer,&encoder.encoderCount,4);

			while(HAL_I2C_Slave_Transmit_IT(&I2cHandle,i2c_tx_buffer,i2c_tx_length) == HAL_BUSY) {	//&I2cHandle, i2c_tx_buffer, i2c_tx_length
				//keep trying
			}

		}
	}
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
		//leds[i].nscale8(ledBrightness[i]);
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

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
#include "stm32f030.h"
#include "ws2812.h"
#include "encoder.h"

int ledBrightness[] = {20,20};
int ledMode[]       = {0,0};
int ledRate[]       = {0,0};
int ledSleep[]      = {0,0};

uint8_t ledRGB[] = {255,0,0};
uint8_t ledHSV[] = {255,255,255};

Ws2812 pixel = Ws2812();
//SoftwareWire encWire = SoftwareWire(ENC_I2C_SDA, ENC_I2C_PORT, ENC_I2C_SCL, ENC_I2C_PORT);
AS5600Encoder encoder = AS5600Encoder();

int main(void)
{
	setup();

	for(;;) {
		encoder.update();
		updateLed();
	}

}

void setup() {
	SystemClock_Config();
	HAL_Init();

	pixel.setup();

	if(encoder.init()) {
		blinkLeds(1,CRGB::Green);
	} else {
		blinkLeds(1,CRGB::Red);
	}

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
				pixel.setPixelColor(i, encoder.getCount()/2, 0, 0);
			} else if (encoder.getCount() <= 1024) {
				pixel.setPixelColor(i, 255 - encoder.getCount()/2, 0, 0);
			} else if (encoder.getCount() <= 1536) {
				pixel.setPixelColor(i, 0, encoder.getCount()/2, 0);
			} else if (encoder.getCount() <= 2048) {
				pixel.setPixelColor(i, 0, 255 - encoder.getCount()/2, 0);
			} else if (encoder.getCount() <= 2560) {
				pixel.setPixelColor(i, 0, 0, encoder.getCount()/2);
			} else if (encoder.getCount() <= 3072) {
				pixel.setPixelColor(i, 0, 0, 255 - encoder.getCount()/2);
			} else if (encoder.getCount() <= 3584) {
				pixel.setPixelColor(i, encoder.getCount()/2, encoder.getCount()/2, encoder.getCount()/2);
			} else if (encoder.getCount() <= 4096) {
				pixel.setPixelColor(i, 255 - encoder.getCount()/2, 255 - encoder.getCount()/2, 255 - encoder.getCount()/2);
			}
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

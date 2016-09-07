#ifndef __main
#define __main

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);


void setup();
void updateLed();
void blinkLeds(int times, uint8_t red, uint8_t green, uint8_t blue);
void blinkLeds(int times, uint32_t colour);

void Configure_GPIO_I2C1(void);
void Configure_I2C1_Slave(void);
void Configure_GPIO_General(void);

void i2c_callback();
void i2c_receive_callback();

void setLedMode(uint8_t, uint8_t);
void setLedBrightness(uint8_t, uint8_t);

//I2C Slave Setup

#define I2C_REQ_REPORT        0
#define I2C_RESET_COUNT       1
#define I2C_SET_ADDR          2
#define I2C_SET_REPORT_MODE   3
#define I2C_CLEAR_EEPROM      4

#define I2C_ENC_LED_PAR_MODE  10
#define I2C_ENC_LED_PAR_BRT   11
#define I2C_ENC_LED_PAR_RATE  12
#define I2C_ENC_LED_PAR_RGB   13
#define I2C_ENC_LED_PAR_HSV   14

//Address configuration
#define WS2812_NUM 		2

const uint8_t i2cBaseAddress = 30;
uint8_t addressOffset;
uint8_t i2cAddress;
int i2c_response_mode = 0;

#define SERIAL_ENABLED

//EEPROM Setup
#define EEPROM_I2C_ADDR 1
#define EEPROM_BRT1_ADDR 2
#define EEPROM_BRT2_ADDR 3
#define EEPROM_MODE1_ADDR 4
#define EEPROM_MODE2_ADDR 5
#define EEPROM_RATE1_ADDR 6
#define EEPROM_RATE2_ADDR 7
#define EEPROM_SLP1_ADDR 8
#define EEPROM_SLP2_ADDR 9
#define EEPROM_RGB1_ADDR 10
#define EEPROM_RGB2_ADDR 11
#define EEPROM_RGB3_ADDR 12
#define EEPROM_HSV1_ADDR 13
#define EEPROM_HSV2_ADDR 14
#define EEPROM_HSV3_ADDR 15

unsigned long lastLoopTime = 0;


#endif

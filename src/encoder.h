#ifndef Encoder_h
#define Encoder_h

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "tm_stm32_gpio.h"
#include "softi2c.h"

//Encoder I2C Setup
#define ENC_I2C_PORT 	GPIOA
#define ENC_I2C_SDA 	GPIO_PIN_5
#define ENC_I2C_SCL 	GPIO_PIN_4
#define ENC_I2C_ADDR	0x36

#define ENC_REG_ZMCO 		0x00
#define ENC_REG_ZPOS1 		0x01
#define ENC_REG_ZPOS2 		0x02
#define ENC_REG_MPOS1 		0x03
#define ENC_REG_MPOS2 		0x04
#define ENC_REG_MANG1 		0x05
#define ENC_REG_MANG2 		0x06
#define ENC_REG_CONF1 		0x07
#define ENC_REG_CONF2 		0x08
#define ENC_REG_ANG_RAW1	0x0C
#define ENC_REG_ANG_RAW2	0x0D
#define ENC_REG_ANG_FIL1	0x0E
#define ENC_REG_ANG_FIL2	0x0F
#define ENC_REG_STATUS		0x0B
#define ENC_REG_AGC			0x1A
#define ENC_REG_MAG1		0x1B
#define ENC_REG_MAG2		0x1C
#define ENC_REG_BURN		0xFF

#define MAG_SIG_GOOD 	0
#define MAG_SIG_MID 	1
#define MAG_SIG_BAD 	2

typedef union{
	volatile long val;
	uint8_t bval[4];
}longByte;



class AS5600Encoder {
	private:

		long count;
		long oldCount;
		long offset;
		long revolutions;
		bool offsetInitialised;


		uint8_t magStrength;


		SoftwareWire encWire = SoftwareWire(ENC_I2C_SDA, ENC_I2C_PORT, ENC_I2C_SCL, ENC_I2C_PORT);
		void readEncoderBytes(uint8_t,uint8_t[],uint8_t);

	public:
		AS5600Encoder();
		bool init();
		void update();

		uint8_t getMagStrength();
		long getCount();

		longByte encoderCount;
		uint8_t statusByte;

};


#endif

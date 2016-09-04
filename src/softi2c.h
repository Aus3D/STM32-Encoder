#ifndef __softi2c
#define __softi2c

#include <stdbool.h>
#include <stdint.h>
#include "timing.h"

//#define I2C_DELAY_TICKS 8	//minimum number of clock ticks in I2C delay function. Delay will be larger than this due to overheads.


class SoftwareWire {
public:
	SoftwareWire(uint16_t sdaPin, GPIO_TypeDef* sdaPort, uint16_t sclPin, GPIO_TypeDef* sclPort);

	void i2cInit(void);
	void i2cDelay(void);
	bool i2cStart(void);
	void i2cStop(void);
	void i2cAck(void);
	void i2cNack(void);
	bool i2cWaitAck(void);
	void i2cSendByte(uint8_t byte);
	uint8_t i2cReceiveByte(void);
	bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
	bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data);
	bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);


private:


	uint16_t _delayLoops;
	uint16_t _sdaPin;
	uint16_t _sclPin;
	GPIO_TypeDef* _sdaPort;
	GPIO_TypeDef* _sclPort;

};

uint8_t pinIndex(uint16_t gpio_pin);

#endif

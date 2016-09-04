#include "softi2c.h"
#include "stm32f0xx.h"

// Software I2C driver

#define I2C_Direction_Transmitter       ((uint16_t)0x0000)
#define I2C_Direction_Receiver          ((uint16_t)0x0400)

#define SCL_H _sclPort->BSRR = _sclPin
#define SCL_L _sclPort->BRR  = _sclPin

#define SDA_H _sdaPort->BSRR = _sdaPin
#define SDA_L _sdaPort->BRR  = _sdaPin

#define SCL_read (_sclPort->IDR & _sclPin)
#define SDA_read (_sdaPort->IDR & _sdaPin)

SoftwareWire::SoftwareWire(uint16_t sdaPin, GPIO_TypeDef* sdaPort, uint16_t sclPin, GPIO_TypeDef* sclPort) {
	_sdaPin = sdaPin;
	_sclPin = sclPin;

	_sdaPort = sdaPort;
	_sclPort = sclPort;

	_delayLoops = 6;
}

void SoftwareWire::i2cInit()
{
	uint8_t sdaPinIndex = pinIndex((uint16_t)_sdaPin);
	uint8_t sclPinIndex = pinIndex((uint16_t)_sclPin);

	_sdaPort->MODER 	= (_sdaPort->MODER & ~((uint32_t)(0x03 << (2 * sdaPinIndex)))) | ((uint32_t)(0x01 << (2 * sdaPinIndex)));
	_sdaPort->OTYPER 	= (_sdaPort->OTYPER & ~(uint16_t)(0x01 << sdaPinIndex)) | ((uint16_t)(0x01 << sdaPinIndex));
	_sdaPort->PUPDR 	= (_sdaPort->PUPDR & ~(0x03 << (2 * sdaPinIndex))) | ((uint32_t)(0x00 << (2 * sdaPinIndex)));
	_sdaPort->OSPEEDR 	= (_sdaPort->OSPEEDR & ~((uint32_t)(0x03 << (2 * sdaPinIndex)))) | ((uint32_t)(0x03 << (2 * sdaPinIndex)));

	_sclPort->MODER 	= (_sclPort->MODER & ~((uint32_t)(0x03 << (2 * sclPinIndex)))) | ((uint32_t)(0x01 << (2 * sclPinIndex)));
	_sclPort->OTYPER 	= (_sclPort->OTYPER & ~(uint16_t)(0x01 << sclPinIndex)) | ((uint16_t)(0x01 << sclPinIndex));
	_sclPort->PUPDR 	= (_sclPort->PUPDR & ~(0x03 << (2 * sclPinIndex))) | ((uint32_t)(0x00 << (2 * sclPinIndex)));
	_sclPort->OSPEEDR 	= (_sclPort->OSPEEDR & ~((uint32_t)(0x03 << (2 * sclPinIndex)))) | ((uint32_t)(0x03 << (2 * sclPinIndex)));
}

bool SoftwareWire::i2cStart(void)
{
    SDA_H;
    SCL_H;
    i2cDelay();
    if (!SDA_read) {
    	return false;
    }
    SDA_L;
    i2cDelay();
    if (SDA_read) {
	    return false;
	}
    SDA_L;
    i2cDelay();
    return true;
}

void SoftwareWire::i2cStop(void)
{
    SCL_L;
    i2cDelay();
    SDA_L;
    i2cDelay();
    SCL_H;
    i2cDelay();
    SDA_H;
    i2cDelay();
}

void SoftwareWire::i2cAck(void)
{
    SCL_L;
    i2cDelay();
    SDA_L;
    i2cDelay();
    SCL_H;
    i2cDelay();
    SCL_L;
    i2cDelay();
}

void SoftwareWire::i2cNack(void)
{
    SCL_L;
    i2cDelay();
    SDA_H;
    i2cDelay();
    SCL_H;
    i2cDelay();
    SCL_L;
    i2cDelay();
}

bool SoftwareWire::i2cWaitAck(void)
{
    SCL_L;
    i2cDelay();
    SDA_H;
    i2cDelay();
    SCL_H;
    i2cDelay();
    if (SDA_read) {
        SCL_L;
        return false;
    }
    SCL_L;
    return true;
}

void SoftwareWire::i2cSendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) {
        SCL_L;
        i2cDelay();
        if (byte & 0x80) {
    	    SDA_H;
        } else {
	        SDA_L;
	    }
        byte <<= 1;
        i2cDelay();
        SCL_H;
        i2cDelay();
    }
    SCL_L;
}

uint8_t SoftwareWire::i2cReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        i2cDelay();
        SCL_H;
        i2cDelay();
        if (SDA_read) {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}



bool SoftwareWire::i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
    int i;
    if (!i2cStart()) {
	    return false;
	}
    i2cSendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!i2cWaitAck()) {
        i2cStop();
        return false;
    }
    i2cSendByte(reg);
    i2cWaitAck();
    for (i = 0; i < len; i++) {
        i2cSendByte(data[i]);
        if (!i2cWaitAck()) {
            i2cStop();
            return false;
        }
    }
    i2cStop();
    return true;
}

bool SoftwareWire::i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!i2cStart()) {
	    return false;
	}
    i2cSendByte((addr << 1) | 0);
    if (!i2cWaitAck()) {
    	i2cStop();
        return false;
    }
    i2cSendByte(reg);
    i2cWaitAck();
    i2cSendByte(data);
    i2cWaitAck();
    i2cStop();
    return true;
}

bool SoftwareWire::i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!i2cStart()) {
	    return false;
	}
    i2cSendByte((addr << 1) | 1);

    if (!i2cWaitAck()) {
    	i2cStop();
        return false;
    }

    while (len) {
        *buf = i2cReceiveByte();
        if (len == 1) {
	        i2cNack();
        } else {
	        i2cAck();
	    }
        buf++;
        len--;
    }
    i2cStop();
    return true;
}


void SoftwareWire::i2cDelay(void) {
	//delayTicks(_delayLoops);

	int time = 2;

	while(time--);
}

uint8_t pinIndex(uint16_t gpio_pin) {
	uint8_t pinVal = 0;
	for (uint8_t pinpos = 0; pinpos < 0x10; pinpos++) {
		if ((gpio_pin & (1 << pinpos)) == 0) {
			continue;
		}
		pinVal = pinpos;
	}
	return pinVal;
}


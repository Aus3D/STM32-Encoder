#include "encoder.h"
#include "softi2c.h"

AS5600Encoder::AS5600Encoder() {


	count = 0;
	oldCount = 0;
	offset = 0;
	revolutions = 0;
	offsetInitialised = false;
}

bool AS5600Encoder::init() {

	encWire.i2cInit();

	if(encWire.i2cStart()) {
		return true;
	} else {
		return false;
	}
}

void AS5600Encoder::update() {
	uint8_t response[2];
	uint8_t mh, ml, md;

	readEncoderBytes(ENC_REG_STATUS,response,1);

	statusByte = response[0];

	md = ((statusByte >> 5) & 0x01);
	ml = ((statusByte >> 4) & 0x01);
	mh = ((statusByte >> 3) & 0x01);

	if(md == 1) {
		magStrength = MAG_SIG_GOOD;
	}

	if(mh == 1) {
		magStrength = MAG_SIG_MID;
	}

	if(ml == 1) {
		magStrength = MAG_SIG_BAD;
	}

	readEncoderBytes(ENC_REG_ANG_FIL1,response,2);

	count = response[0];
	count = count << 8;
	count |= response[1];

	//check if we've moved from one revolution to the next
	if((count-oldCount) > 2048) {
		revolutions -= 1;
	} else if((oldCount - count) > 2048) {
		revolutions += 1;
	}

	oldCount = count;

	//make the starting position 'zero'
	if(offsetInitialised == false) {
		offset = -count;
		offsetInitialised = true;
	}

	encoderCount.val = (revolutions * 4092) + (count + offset);
}

void AS5600Encoder::readEncoderBytes(uint8_t startAddr, uint8_t buf[], uint8_t numBytes) {
	encWire.i2cStart();

	encWire.i2cSendByte((ENC_I2C_ADDR << 1) | 0);

	if (!encWire.i2cWaitAck()) {
		encWire.i2cStop();
	}

	encWire.i2cSendByte(startAddr);

	if (!encWire.i2cWaitAck()) {
		encWire.i2cStop();
	}

	encWire.i2cStart();

	encWire.i2cSendByte((ENC_I2C_ADDR << 1) | 1);

	if (!encWire.i2cWaitAck()) {
		encWire.i2cStop();
	}

	for(int i = 0; i < numBytes; i++) {
		buf[i] = encWire.i2cReceiveByte();

		if (!encWire.i2cWaitAck()) {
			encWire.i2cStop();
		}
	}

	encWire.i2cStop();

}

uint8_t AS5600Encoder::getMagStrength() {
	return magStrength;
}

long AS5600Encoder::getCount() {
	return count;
}

#include "encoder.h"
#include "softi2c.h"

AS5600Encoder::AS5600Encoder() {

	count = 0;
	oldCount = 0;
	offset = 0;
	revolutions = 0;
	offsetInitialised = false;
	magStrength = MAG_SIG_BAD;
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
	uint8_t mh, ml, md;

	//read and store all data from encoder
	encWire.i2cRead(ENC_I2C_ADDR,ENC_REG_STATUS,ENC_REG_AGC - ENC_REG_STATUS + 1,rawData + ENC_REG_STATUS);

	md = !CHECK_BIT(rawData[ENC_REG_STATUS],ENC_STATUS_BIT_MD);
	ml = !CHECK_BIT(rawData[ENC_REG_STATUS],ENC_STATUS_BIT_ML);
	mh = !CHECK_BIT(rawData[ENC_REG_STATUS],ENC_STATUS_BIT_MH);

	//TODO should replace this with checking the magnitude of the AGC register.
	if(md) {
		magStrength = MAG_SIG_GOOD;
	}

	if(ml) {
		magStrength = MAG_SIG_BAD;
	}

	//convert raw bytes to long containing angular reading
	count = rawData[ENC_REG_ANG_FIL1];
	count = count << 8;
	count |= rawData[ENC_REG_ANG_FIL2];

	//check if we've moved from one revolution to the next
	if((count-oldCount) > 2048) {
		revolutions -= 1;
	} else if((oldCount - count) > 2048) {
		revolutions += 1;
	}

	oldCount = count;

	//if first loop, make the starting position 'zero'
	if(offsetInitialised == false) {
		offset = -count;
		offsetInitialised = true;
	}

	encoderCount.val = (revolutions * 4092) + (count + offset);
}

uint8_t AS5600Encoder::getMagStrength() {
	return magStrength;
}

long AS5600Encoder::getCount() {
	return count;
}

void AS5600Encoder::setZeroed() {
	setOffset(count);
}

long AS5600Encoder::getOffset() {
	return this->offset;
}

void AS5600Encoder::setOffset(long offset) {
	this->offset = offset;
}

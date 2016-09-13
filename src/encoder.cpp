#include "encoder.h"
//#include "soft_i2c.h"
#include "softi2c.h"


AS5600Encoder::AS5600Encoder() {


	count = 0;
	oldCount = 0;
	offset = 0;
	revolutions = 0;
	offsetInitialised = false;
	statusByte = 0;
	agcByte = 0;
	magStrength = MAG_SIG_BAD;
}

bool AS5600Encoder::init() {

	//soft_i2c_init();

	encWire.i2cInit();

	if(encWire.i2cStart()) {
		return true;
	} else {
		return false;
	}

}

void AS5600Encoder::update() {
	uint8_t mh, ml, md;

	//readEncoderBytes(ENC_REG_STATUS,rawData + ENC_REG_STATUS,ENC_REG_AGC - ENC_REG_STATUS);
	encWire.i2cRead(ENC_I2C_ADDR,ENC_REG_STATUS,ENC_REG_AGC - ENC_REG_STATUS + 1,rawData + ENC_REG_STATUS);
	//encWire.i2cRead(ENC_I2C_ADDR,ENC_REG_STATUS,1,rawData + ENC_REG_STATUS);
	//encWire.i2cRead(ENC_I2C_ADDR,ENC_REG_AGC,1,rawData + ENC_REG_AGC);
	//encWire.i2cRead(ENC_I2C_ADDR,ENC_REG_ANG_FIL1,2,rawData + ENC_REG_ANG_FIL1);

	//soft_i2c_request_bytes(ENC_I2C_ADDR << 1,ENC_REG_ANG_FIL1,rawData + ENC_REG_ANG_FIL1,2,true);

	statusByte = rawData[ENC_REG_STATUS];

	md = !CHECK_BIT(statusByte,ENC_STATUS_BIT_MD);
	ml = !CHECK_BIT(statusByte,ENC_STATUS_BIT_ML);
	mh = !CHECK_BIT(statusByte,ENC_STATUS_BIT_MH);

	if(md) {
		magStrength = MAG_SIG_GOOD;
	}

	if(ml) {
		magStrength = MAG_SIG_BAD;
	}

	agcByte = rawData[ENC_REG_AGC];

	count = rawData[ENC_REG_ANG_FIL1];//angleFiltered[0];
	count = count << 8;
	count |= rawData[ENC_REG_ANG_FIL2];//angleFiltered[1];

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

uint8_t AS5600Encoder::readEncoderByte(uint8_t startAddr) {
	uint8_t buffer[1];

	//readEncoderBytes(startAddr,buffer,1);

	//encWire.i2cRead(ENC_I2C_ADDR,startAddr,1,buffer);

	return buffer[0];
}

void AS5600Encoder::readEncoderBytes(uint8_t startAddr, uint8_t buf[], uint8_t numBytes) {

	//encWire.i2cRead(ENC_I2C_ADDR,startAddr,numBytes,buf);

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

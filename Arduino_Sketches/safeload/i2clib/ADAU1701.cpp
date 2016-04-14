/*
 * ADAU1701.cpp
 *
 *  Created on: 15.01.2016
 *      Author: Fabian Geißler (fabian.geissler[at]tu-dresden.de)
 */

#include "ADAU1701.h"
#include "ADAU1701_REGISTERS.h"

#include "I2CMaster.h"

void ADAU1701::writeBuffer(uint8_t *data, uint16_t size)
{
	while(size > 0) {	
        mI2C->write(*(data++));
        --size;
    }
}

void ADAU1701::readBuffer(uint8_t *data, uint16_t size)
{
	while(size > 0) {	
        *(data++) = mI2C->read();
        --size;
    }
}

ADAU1701::ADAU1701(I2CMaster &i2c, uint8_t address) : mI2CAddress((address | ADAU_I2C_ADDRESS) >> 1), mI2C(&i2c), mComOK(true)
{
	available();
}

bool ADAU1701::available()
{	
	uint8_t ccr[ADAU_CC_WIDTH];
	mComOK = read(ADAU_CC, ADAU_CC_WIDTH, ccr); // perform a read to check connection

	return mComOK;
}

bool ADAU1701::write(uint16_t addr, uint16_t size, uint8_t *data) 
{
	mI2C->startWrite(mI2CAddress);

	mI2C->write(addr >> 8);
	mI2C->write(addr & 0xFF);

	writeBuffer(data, size);
	
	mComOK = mI2C->endWrite();

	return mComOK;
}

bool ADAU1701::read(uint16_t addr, uint16_t size, uint8_t *data) {
	mI2C->startWrite(mI2CAddress);

	mI2C->write(addr >> 8);
	mI2C->write(addr & 0xFF);

	mComOK = mI2C->startRead(mI2CAddress, size);

	if(!mComOK)
	{	
		return false;	
	}
	
	readBuffer(data, size);

	return true;
}

bool ADAU1701::writeWithCallback(uint16_t addr, ADAUMemoryCallback cb, uint16_t size, uint8_t blocksize) {
	if (cb == 0)
		return false;

	mI2C->startWrite(mI2CAddress);

	mI2C->write(addr >> 8);
	mI2C->write(addr & 0xFF);

	uint8_t data[blocksize];
	uint8_t read_cnt = blocksize;

	while (size != 0) {
		if (size < blocksize)
			read_cnt = size;

		cb(data, read_cnt);
		writeBuffer(data, read_cnt);
		size -= read_cnt;
	}

	mComOK = mI2C->endWrite();

	return mComOK;
}

bool ADAU1701::readWithCallback(uint16_t addr, ADAUMemoryCallback cb, uint16_t size, uint8_t blocksize) {
	if (cb == 0)
		return false;

	mI2C->startWrite(mI2CAddress);

	mI2C->write(addr >> 8);
	mI2C->write(addr & 0xFF);

	mComOK = mI2C->startRead(mI2CAddress, size);

	if(!mComOK)
	{	
		return false;	
	}

	uint8_t data[blocksize];
	uint8_t write_cnt = blocksize;

	while (size != 0) {
		if (size < blocksize)
			write_cnt = size;

		readBuffer(data, write_cnt);
		cb(data, write_cnt);
		size -= write_cnt;
	}

	return true;
}

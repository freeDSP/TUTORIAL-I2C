/*
 * ADAU1701.h
 *
 *  Created on: 15.01.2016
 *      Author: Fabian Geißler (fabian.geissler[at]tu-dresden.de)
 */

#include <stdint.h>
#include <stdbool.h>

#ifndef ADAU1701_H
#define ADAU1701_H

    class I2CMaster;

    typedef uint8_t(*ADAUMemoryCallback)(uint8_t *data, uint8_t size);

    class ADAU1701 {
    public:
        ADAU1701(I2CMaster &i2c, uint8_t address); // address: ADDR1:ADDR0 -> 0..3

	    bool write(uint16_t addr, uint16_t size, uint8_t *data);
	    bool read(uint16_t addr, uint16_t size, uint8_t *data);
	    bool writeWithCallback(uint16_t addr, ADAUMemoryCallback cb, uint16_t size, uint8_t blocksize);
	    bool readWithCallback(uint16_t addr, ADAUMemoryCallback cb, uint16_t size, uint8_t blocksize);

        bool available();

    private:
        I2CMaster *mI2C;
        bool mComOK;
        uint8_t mI2CAddress;

        void writeBuffer(uint8_t *data, uint16_t size);
        void readBuffer(uint8_t *data, uint16_t size);
    };

#endif

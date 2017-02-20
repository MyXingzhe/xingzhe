#ifndef __XZ_H__
#define __XZ_H__

#include <mraa.hpp>

class XZIO: public mraa::I2c {
public:
	XZIO(int bus, bool raw):mraa::I2c(bus, raw){};
	int8_t readBit(uint8_t reg, uint8_t bits, uint8_t *data);
	int8_t readBits(uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t *data);
	int8_t readBytes(uint8_t reg, uint8_t length, uint8_t *data);

	bool writeBit(uint8_t reg, uint8_t bits, uint8_t data);
	bool writeBits(uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data);
	bool writeBytes(uint8_t reg, uint8_t length, uint8_t *data);
};

#endif

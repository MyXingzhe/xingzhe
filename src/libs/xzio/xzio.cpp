#include "xzio.h"

int8_t XZIO::readBit(uint8_t reg, uint8_t bits, uint8_t *data) {
    uint8_t value = readReg(reg);
    *data = value & (1 << bits);
    return 0;
}

int8_t XZIO::readBits(uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t *data) {
    uint8_t value = readReg(reg);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    value &= mask;
    value >>= (bitStart - length + 1);
    *data = value;

    return 0;
}

int8_t XZIO::readBytes(uint8_t reg, uint8_t length, uint8_t *data) {
	return readBytesReg (reg, data, length);
}


bool XZIO::writeBit(uint8_t reg, uint8_t bits, uint8_t data) {
    uint8_t value = readReg(reg);
    value = (data != 0) ? (value | (1 << bits)) : (value & ~(1 << bits));
    return writeReg (reg, value);
}

bool XZIO::writeBits(uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data) {
    uint8_t value = readReg(reg);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    value &= ~(mask); // zero all important bits in existing byte
    value |= data; // combine data with existing byte
    return writeReg(reg, value);

}

bool XZIO::writeBytes(uint8_t reg, uint8_t length, uint8_t *data) {
    uint8_t buf[128];

    if (length > 127) {
        return false;
    }

    buf[0] = reg;
    memcpy(buf+1, data, length);
    return write(buf, length+1);
}


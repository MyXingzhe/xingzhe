#ifndef __I2C_FUNC_H__
#define __I2C_FUNC_H__

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include "ros/ros.h"

class I2cFunc
{
public:
	I2cFunc(uint8_t bus, uint8_t addr);
	~I2cFunc();

	static int I2cWriteByteData(uint8_t bus, uint8_t addr, uint8_t command, uint8_t value);
	static int I2cWriteByte(uint8_t bus, uint8_t addr, uint8_t value);
	static int I2cWriteBit(uint8_t bus, uint8_t addr, uint8_t cmd, uint8_t bits, uint8_t data);
	static int I2cWriteBits(uint8_t bus, uint8_t addr, uint8_t cmd, uint8_t bit_start, uint8_t length, uint8_t data);

	static uint8_t I2cReadByteData(uint8_t bus, uint8_t addr, uint8_t command);
	static uint8_t I2cReadBit(uint8_t bus, uint8_t addr, uint8_t cmd, uint8_t bits, uint8_t *data);
	static int8_t  I2cReadBits(uint8_t bus, uint8_t addr, uint8_t cmd, uint8_t bit_start, uint8_t length, uint8_t *data) ;

	static int I2cReadByte(uint8_t bus, uint8_t addr);

	static int I2cOpen(uint8_t bus, uint8_t addr);
	static int I2cClose(int fd);

private:
	static int I2cSmbusAccess(int fd, char read_write, uint8_t command, 
                                     int size, union i2c_smbus_data *data);

public:

private:

};

#endif // __I2CFUNC_H__
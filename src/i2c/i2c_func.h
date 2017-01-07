#ifndef __I2C_FUNC_H__
#define __I2C_FUNC_H__

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

class I2cFunc
{
public:
	I2cFunc();
	~I2cFunc();

	int I2cWriteByte(int file, unsigned char command, unsigned char value);
	int I2cReadByte(int file, unsigned char command);

	int I2cOpen(unsigned char bus, unsigned char addr);

private:
	int I2cSmbusAccess(int file, char read_write, unsigned char command, 
                                     int size, union i2c_smbus_data *data);


};

#endif // __I2CFUNC_H__
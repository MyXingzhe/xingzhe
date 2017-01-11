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
	I2cFunc(unsigned char bus, unsigned char addr);
	~I2cFunc();

	int I2cWriteByteData(unsigned char command, unsigned char value);
	int I2cWriteByte(unsigned char value);

	int I2cReadByteData(unsigned char command);
	int I2cReadByte();

	int I2cOpen();

private:
	int I2cSmbusAccess(int fd, char read_write, unsigned char command, 
                                     int size, union i2c_smbus_data *data);

public:

private:
	int m_fd;
	unsigned char m_bus;
	unsigned char m_addr;

};

#endif // __I2CFUNC_H__
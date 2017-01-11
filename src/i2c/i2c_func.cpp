#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include "i2c_func.h"

I2cFunc::I2cFunc(unsigned char bus, unsigned char addr)
{
	m_bus = bus;
	m_addr = addr;
	m_fd = -1;
}

I2cFunc::~I2cFunc()
{
	if(m_fd != -1)
		close(m_fd);
}

int I2cFunc::I2cSmbusAccess(int fd, char read_write, unsigned char command, 
                                     int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(fd, I2C_SMBUS, &args);
}

int I2cFunc::I2cReadByteData(unsigned char command)
{
	union i2c_smbus_data data;
	if (I2cSmbusAccess(m_fd, I2C_SMBUS_READ,command,
	                     I2C_SMBUS_BYTE_DATA,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

int I2cFunc::I2cWriteByteData(unsigned char command, unsigned char value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return I2cSmbusAccess(m_fd, I2C_SMBUS_WRITE,command,
	                        I2C_SMBUS_BYTE_DATA, &data);
}

int I2cFunc::I2cReadByte()
{
	union i2c_smbus_data data;
	if (I2cSmbusAccess(m_fd, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

int I2cFunc::I2cWriteByte(unsigned char value)
{
	return I2cSmbusAccess(m_fd, I2C_SMBUS_WRITE, value,
	                        I2C_SMBUS_BYTE,NULL);
}

int I2cFunc::I2cOpen()
{
    int file;
    char filename[16];
    sprintf(filename,"/dev/i2c-%d", m_bus);
    if ((file = open(filename,O_RDWR)) < 0)
    {
        return(file);
    }

    if (ioctl(file, I2C_SLAVE, m_addr) < 0)
    {
        return(-1);
    }

    return(file);
}
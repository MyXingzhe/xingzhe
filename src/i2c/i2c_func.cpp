#include <stdio.h>
#include <iostream>
#include "i2c_func.h"

I2cFunc::I2cFunc()
{

}

I2cFunc::~I2cFunc()
{
	
}

int I2cFunc::I2cWriteByte(int file, unsigned char command, unsigned char value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return I2cSmbusAccess(file,I2C_SMBUS_WRITE,command,
	                        I2C_SMBUS_BYTE_DATA, &data);
}


int I2cFunc::I2cSmbusAccess(int file, char read_write, unsigned char command, 
                                     int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file,I2C_SMBUS,&args);
}

int I2cFunc::I2cReadByte(int file, unsigned char command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
	                     I2C_SMBUS_BYTE_DATA,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

int I2cFunc::I2cOpen(unsigned char bus, unsigned char addr)
{
    int file;
    char filename[16];
    sprintf(filename,"/dev/i2c-%d", bus);
    if ((file = open(filename,O_RDWR)) < 0)
    {
        return(file);
    }

    if (ioctl(file,I2C_SLAVE,addr) < 0)
    {
        return(-1);
    }

    return(file);
}
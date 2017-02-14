#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>

#include "i2c_func.h"

I2cFunc::I2cFunc(uint8_t bus, uint8_t addr)
{
}

I2cFunc::~I2cFunc()
{
}

int I2cFunc::I2cOpen(uint8_t bus, uint8_t addr)
{
    int file;
    char filename[16];
    sprintf(filename,"/dev/i2c-%d", bus);
    if ((file = open(filename,O_RDWR)) < 0)
    {
        return(file);
    }

    if (ioctl(file, I2C_SLAVE, addr) < 0)
    {
        return(-1);
    }

    return(file);
}

int I2cFunc::I2cClose(int m_fd)
{
    close(m_fd);
}

int I2cFunc::I2cSmbusAccess(int fd, char read_write, uint8_t command, 
                                     int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(fd, I2C_SMBUS, &args);
}

uint8_t I2cFunc::I2cReadByteData(uint8_t bus, uint8_t addr, uint8_t command)
{
    int ret, m_fd;
    union i2c_smbus_data data;

    m_fd = I2cOpen(bus, addr);

	ret = I2cSmbusAccess(m_fd, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA,&data);

    I2cClose(m_fd);
    if(ret)
		return -1;
	else
		return 0x0FF & data.byte;
}

int I2cFunc::I2cWriteByteData(uint8_t bus, uint8_t addr, uint8_t command, uint8_t value)
{
    int ret, m_fd;
    union i2c_smbus_data data;

    m_fd = I2cOpen(bus, addr);

	data.byte = value;
	ret = I2cSmbusAccess(m_fd, I2C_SMBUS_WRITE, command,
	                        I2C_SMBUS_BYTE_DATA, &data);
    I2cClose(m_fd);

    return ret;
}

int I2cFunc::I2cReadByte(uint8_t bus, uint8_t addr)
{
    int m_fd = 0;
	union i2c_smbus_data data;
	if (I2cSmbusAccess(m_fd, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

int I2cFunc::I2cWriteByte(uint8_t bus, uint8_t addr, uint8_t value)
{
    int m_fd = 0;
	return I2cSmbusAccess(m_fd, I2C_SMBUS_WRITE, value,
	                        I2C_SMBUS_BYTE,NULL);
}

uint8_t I2cFunc::I2cReadBit(uint8_t bus, uint8_t addr, uint8_t cmd, uint8_t bits, uint8_t *data)
{
	uint8_t value = I2cReadByteData(bus, addr, cmd);
	*data = value & (1 << bits);

	return 0;
}


int I2cFunc::I2cWriteBit(uint8_t bus, uint8_t addr, uint8_t cmd, uint8_t bits, uint8_t data)
{
    int m_fd = 0;
	uint8_t value = I2cReadByteData(bus, addr, cmd);
    value = (data != 0) ? (value | (1 << bits)) : (value & ~(1 << bits));

	return I2cSmbusAccess(m_fd, I2C_SMBUS_WRITE, value,
	                        I2C_SMBUS_BYTE,NULL);
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2cFunc::I2cReadBits(uint8_t bus, uint8_t addr, uint8_t cmd, uint8_t bit_start, uint8_t length, uint8_t *data) 
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, value;
    value = I2cReadByteData(bus, addr, cmd);

    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
    value &= mask;
    value >>= (bit_start - length + 1);
    *data = value;

    return 0;
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
int I2cFunc::I2cWriteBits(uint8_t bus, uint8_t addr, uint8_t cmd, uint8_t bit_start, uint8_t length, uint8_t data) 
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t value = I2cReadByteData(bus, addr, cmd);
    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
    data <<= (bit_start - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    value &= ~(mask); // zero all important bits in existing byte
    value |= data; // combine data with existing byte
    
    return I2cWriteByteData(bus, addr, cmd, value);
    
}


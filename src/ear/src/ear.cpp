#include "ros/ros.h"

#include "ear.h"


Ear::Ear()
{
	m_i2c = new mraa::I2c(1);
	if(m_i2c == NULL) {
		ROS_DEBUG("Ear Constructor Failed!");
		return;
	}

	cnt = 0;

	ROS_DEBUG("Ear Constructor Successfully");
}


Ear::~Ear()
{

}

uint32_t Ear::DoListen()
{
	uint32_t dist_hi, dist_lo;
	uint8_t rx_tx_buf[2];

	if(cnt == 0) {
		cnt++;
	} else {
		/* i don't care the first time */
		m_i2c->address(USONIC_0_ADDR);
		dist_hi = m_i2c->readReg(2);

		m_i2c->address(USONIC_0_ADDR);
		dist_lo = m_i2c->readReg(3);
	}

	/* this is used for next time */
	m_i2c->address(USONIC_0_ADDR);
	rx_tx_buf[0] = 2;
	rx_tx_buf[1] = CMD_DETECT_0_5_METER;
	m_i2c->write(rx_tx_buf, 2);

 	return dist_hi*255 + dist_lo;
}

int Ear::EarInit(int connector, int pin)
{
	m_connector = connector;
	m_pin = pin;
	
	return 0;
}
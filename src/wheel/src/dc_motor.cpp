#include <iostream>
#include <time.h>
#include <stdint.h>

#include "ros/ros.h"
#include "dc_motor.h"

#include "prupwm/prupwm_duty.h"
#include "prupwm/prupwm_period.h"

DCMotor::DCMotor(uint32_t channel, uint32_t connector, uint32_t pin)
{
	m_channel = channel;
	m_dir = DIR_POSITIVE;
	m_connector = connector;
	m_pin = pin;

}

DCMotor::~DCMotor()
{

}

int DCMotor::Init()
{
	return 0;
}

int DCMotor::Start()
{
	return 0;
}

int DCMotor::Stop()
{
	return 0;
}

int DCMotor::SetDir(int dir)
{
	return 0;
}

int DCMotor::SetSpeed(uint32_t speed)
{
	return 0;
}

int DCMotor::SpeedUp()
{
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<prupwm::prupwm_duty>("SetDuty");
	prupwm::prupwm_duty srv;
	srv.request.channel = 0;
	srv.request.duty = 0.0;
	if (client.call(srv))
	{
		ROS_INFO("OK");
	}
	else
	{
		ROS_ERROR("");
		return 1;
	}

	return 0;
}

int DCMotor::SpeedDown()
{
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<prupwm::prupwm_duty>("SetDuty");
	prupwm::prupwm_duty srv;
	srv.request.channel = 0;
	srv.request.duty = 0.0;
	if (client.call(srv))
	{
		ROS_INFO("OK");
	}
	else
	{
		ROS_ERROR("");
		return 1;
	}
	return 0;
}

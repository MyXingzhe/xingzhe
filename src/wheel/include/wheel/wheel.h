#include "pruPWM.h"

class Wheel
{
public:
	Wheel();
	~Wheel();

private:

public:

private:
	PRU *m_left_pwm;
	PRU *m_right_pwm;
};

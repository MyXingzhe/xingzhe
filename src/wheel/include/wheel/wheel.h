#include "prupwm.h"

class Wheel
{
public:
	Wheel();
	~Wheel();

private:

public:

private:
	PRUPWM *m_left_pwm;
	PRUPWM *m_right_pwm;
};

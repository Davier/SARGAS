#pragma once

#include <string>

class PWM {
public:
	PWM(unsigned int pwmchip, unsigned int pwm);
	void setFrequency(float Hz); // Changing the frequency stops the PWM and clears the duty cycle
	void setDuty(float percent);
	void start();
	void stop();

private:
	std::string m_directory;
};
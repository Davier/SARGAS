#include "PWM.hpp"
#include "SysFsHelper.hpp"

#include <sstream>

using namespace sysfs;
using namespace std;

PWM::PWM(unsigned int pwmchip, unsigned int pwm) {
	stringstream export_file;
	export_file <<  "/sys/class/pwm/pwmchip" << pwmchip << "/export";
	writeFile(export_file.str(), pwm);
	stringstream directory;
	directory <<  "/sys/class/pwm/pwmchip" << pwmchip << "/pwm" << pwm;
	m_directory = directory.str();
}

void PWM::start()
{
	writeFile(m_directory + "/enable", "1");
}

void PWM::stop()
{
	writeFile(m_directory + "/enable", "0");
}

void PWM::setFrequency(float Hz) {
	stop();
	setDuty(0.0f);
	writeFile<unsigned long>(m_directory + "/period", 1.0f / Hz * 1000000000.0f); // Write period in ns
}

void PWM::setDuty(float percent) {
	unsigned long period = readFile<unsigned long>(m_directory + "/period"); // Read period in ns
	writeFile<unsigned long>(m_directory + "/duty_cycle", percent * static_cast<float>(period) / 100.0f); // Write duty in ns
}

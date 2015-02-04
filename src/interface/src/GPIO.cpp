#include "GPIO.hpp"

#include "SysFsHelper.hpp"
#include <sstream>
#include <stdexcept>

using namespace std;
using namespace sysfs;

GPIO::GPIO(unsigned int id, bool direction_in) : m_id{ id },m_direction_in{direction_in} {
	stringstream value_file;
	value_file <<  "/sys/class/gpio/gpio" << m_id << "/value";
	m_value_file = value_file.str();
	writeFile("/sys/class/gpio/export", m_id);
	setDirection(m_direction_in);
}

GPIO::~GPIO() {
	writeFile("/sys/class/gpio/unexport", m_id);
}

void GPIO::setDirection(bool direction_in) {
	m_direction_in = direction_in;
	string direction;
	if (m_direction_in) {
		direction = "in";
	}
	else {
		direction = "out";
	}
	stringstream file;
	file << "/sys/class/gpio/gpio";
	file << m_id << "/direction";
	writeFile(file.str(), direction);
}

void GPIO::setValue(bool value) {
	if (!m_direction_in) {
		throw runtime_error("Try to set input GPIO " + m_id);
	}
	writeFile(m_value_file, value);
}

bool GPIO::getValue()
{
	return readFile<bool>(m_value_file);
}

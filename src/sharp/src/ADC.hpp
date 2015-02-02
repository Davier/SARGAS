#pragma once

#include <string>

class ADC {
public:
	ADC(unsigned int id);
	unsigned int getValue();

private:
	std::string m_input_file;
};

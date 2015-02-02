#pragma once

#include <string>

class GPIO {
public:
	GPIO(unsigned int id, bool direction_in);
	~GPIO();
	void setDirection(bool direction_in);
	void setValue(bool value);
	bool getValue();

private:
	const unsigned int m_id;
	std::string m_value_file;
	bool m_direction_in;
};
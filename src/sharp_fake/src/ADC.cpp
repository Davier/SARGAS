#include "ADC.hpp"
#include "SysFsHelper.hpp"

#include <sstream>

using namespace std;
using namespace sysfs;

ADC::ADC(unsigned int id) {
	stringstream input_file;
	input_file << "/sys/devices/ocp.*/44e0d000.tscadc/TI-am335x-adc/iio\\:device0/in_voltage" << id << "_raw";
	m_input_file = resolveFilePath(input_file.str());
}

unsigned int ADC::getValue() {
	return readFile<unsigned int>(m_input_file);
}

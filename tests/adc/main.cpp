#include <iostream>
#include <fstream>
#include <thread>

using namespace std;

/* Test to read ADC
 *   Requires: cape cape-bone-iio imported
 *             path to adc file
 */

int main() {
	auto adc_path = "/sys/devices/ocp.3/helper.13/AIN1"; // AIN0 is pin P9_40 (VADC is P9_32 and AGND is P9_34)
	ifstream adc_file(adc_path);
	while(1) {
		int value;
		adc_file >> value;
		adc_file.seekg(0);
		cout << "Value : " << value << endl;
		this_thread::sleep_for(chrono::milliseconds(100));
	}
	return 0;
}

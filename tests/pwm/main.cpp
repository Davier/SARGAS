#include <iostream>
#include <fstream>
#include <thread>

using namespace std;

/* Test to generate PWM
 *   Requires: cape bone_pwm_P8_13 imported
 *             path to pwm file
 */

int main() {
	string pwm_path = "/sys/devices/ocp.3/pwm_test_P8_13.11/"; // EHRPWM2B is pin P8_13
	ofstream pwm_run(pwm_path + "run");
	if(!pwm_run.is_open()) {
		cout << "Could not open file, are you root ?" << endl;
		return -1;
	}
	ofstream pwm_period(pwm_path + "period");
	ofstream pwm_duty(pwm_path + "duty");
	ofstream pwm_polarity(pwm_path + "polarity");

	auto period = 20000000;

	
	pwm_run.seekp(0);
	pwm_run << "0";
	pwm_run.flush();
	pwm_period.seekp(0);
	pwm_period << period; // in ns
	pwm_period.flush();
	pwm_duty.seekp(0);
	pwm_duty << 0; // in ns
	pwm_duty.flush();
	pwm_polarity.seekp(0);
	pwm_polarity << "0"; // duty is high
	pwm_polarity.flush();
	pwm_run.seekp(0);
	pwm_run << 1;
	pwm_run.flush();

	while(1) {
		//for(auto duty = 5.0f; duty <= 10.0f; duty += 1.0f) {
		string input;
		while(input != "q") {
			pwm_duty.seekp(0);
			pwm_duty << (unsigned int) (period * duty / 100.0f);
			pwm_duty.flush();
			//cout << "Duty : " << duty << "%" << endl;
			//this_thread::sleep_for(chrono::milliseconds(1000));
			getline(cin, input)
		}
	}
	
	return 0;
}

#include <iostream>
#include <fstream>
#include <thread>

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

using namespace std;

int main() {
	// The path here should be detected since the numbers will change at reboot
	auto adc_path = "/sys/devices/ocp.3/helper.13/AIN0"; // AIN0 is pin P9_40 (VADC is P9_32 and AGND is P9_34)
	ifstream adc_file(adc_path);

	// Setup ROS node and message
	ros::NodeHandle nh;
	sensor_msgs::Range range_msg;
	ros::Publisher pub_range( "range_data", &range_msg);
	nh.initNode();
	nh.advertise(pub_range);
	range_msg.radiation_type = sensor_msgs::Range::INFRARED;
	range_msg.header.frame_id = "/sharp"
	range_msg.field_of_view = 0.01;
	range_msg.min_range = 0.03;
	range_msg.max_range = 0.4;

	while(true) {
		// Get sensor value
		int value;
		adc_file >> value;
		adc_file.seekg(0);
		// Configure and publish ROS message
		range_msg.range = (float) value; // Here convert to meters
		range_msg.header.stamp = nh.now();
		pub_range.publish(&range_msg);
		nh.spinOnce();
		// Wait 100ms
		this_thread::sleep_for(chrono::milliseconds(100));
	}
	return 0;
}


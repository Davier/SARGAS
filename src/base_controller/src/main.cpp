#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "PWM.hpp"
#include "SysFsHelper.hpp"

geometry_msgs::Twist::ConstPtr last_command;
bool last_command_dirty = false;
void commandReceivedCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	ROS_INFO("Callback ! l: %f / a: %f", msg->linear.x, msg->angular.z);
	last_command = msg;
	last_command_dirty = true;
}

void startFlightRegulator(PWM &motor) {
	ros::Duration time_step(0.5f);
	float percent_step = 1.0f;

	float percent = 10.0f;
	while(percent < 20.0f) {
		percent += percent_step;
		motor.setDuty(percent);
		time_step.sleep();
	}
	motor.setDuty(10.0f);
}

int main(int argc, char **argv) {
	try {
		// TODO: check why ros catches every exception ... meanwhile this must stay before ros::init()
		PWM motor_linear(0, 1); // P9_22
		motor_linear.setFrequency(50);
		motor_linear.setDuty(5.0f);
		motor_linear.start();

		PWM servo_angular(0, 0); // P9_21
		servo_angular.setFrequency(50);
		servo_angular.setDuty(7.5f);
		servo_angular.start();

		ros::init(argc, argv, "base_controller", ros::init_options::NoSigintHandler);
		ros::NodeHandle nh;
		ros::Subscriber command_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 5, commandReceivedCallback); // Subscribe with a buffer of 5 messages
	
		startFlightRegulator(motor_linear);
	
		ros::Rate loop_rate(100); // Execute loop at 100Hz
		while(ros::ok()) {
			if(last_command_dirty) {
				last_command_dirty = false;
				motor_linear.setDuty(100.0f * last_command->linear.x);
				ROS_INFO("Set duty to %f", 100.0f * last_command->linear.x);
				//servo_angular.setDuty(
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	catch (std::exception &e) {
		ROS_ERROR("Exception : %s", e.what());
	}
	return 0;
}

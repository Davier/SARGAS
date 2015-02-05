#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "GPIO.hpp"
#include "PWM.hpp"
#include "SysFsHelper.hpp"

geometry_msgs::Twist::ConstPtr last_command;
bool last_command_dirty = false;
bool CloseRange=false;
const bool in=true;
const bool out=false;
void commandReceivedCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	ROS_INFO("Commands: l: %f / a: %f", msg->linear.x, msg->angular.z);
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

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& lasermsg)
{
  for(int i=0;i<3;i++)
	{
  		if(lasermsg->ranges[i]<0.3f)
		{
			CloseRange=true;
			break;
		}
		else
		{
			CloseRange=false;
		}
	}
}

int main(int argc, char **argv) {
	try {
		// TODO: check why ros catches every exception ... meanwhile this must stay before ros::init()
		const float max_right = 8.75;
		const float max_left = 6.25f;
		const float zero = 7.582f;
		const float coef_angle = 0.60f;

		PWM motor_linear(4, 1); // P8_13
		motor_linear.setFrequency(50);
		motor_linear.setDuty(5.0f);
		motor_linear.start();

		PWM servo_angular(4, 0); // P8_19
		servo_angular.setFrequency(50);
		servo_angular.setDuty(zero);
		servo_angular.start();

		GPIO DeadMan(15,in);		

		ros::init(argc, argv, "base_controller");
		ros::NodeHandle nh;
		ros::Subscriber command_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 5, commandReceivedCallback); // Subscribe with a buffer of 2 messages
		ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 5, LaserCallback);//suscribe to scan to stop the robot if it's to close to something
		//startFlightRegulator(motor_linear);
		
		ros::Rate loop_rate(100); // Execute loop at 100Hz
		while(ros::ok()) {
			if(last_command_dirty) {
				last_command_dirty = false;
				float duty_linear = 5.0f;
				if((last_command->linear.x > 0.0f)&&(!DeadMan.getValue())&&(!CloseRange)) {
					duty_linear = 5.50f;
				}
				motor_linear.setDuty(duty_linear);
				float order_angle = coef_angle * last_command->angular.z;
				if(order_angle > 1.0) {
					order_angle = 1.0;
				}
				else if(order_angle < -1.0) {
					order_angle = -1.0;
				}
				float duty_angular = zero + (max_left - zero) * order_angle;
				servo_angular.setDuty(duty_angular);
				ROS_INFO("Duty: l: %f / a: %f", duty_linear, duty_angular);
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	motor_linear.setDuty(5.0f);
	}
	catch (std::exception &e) {
		ROS_ERROR("Exception : %s", e.what());
	}
	return 0;
}

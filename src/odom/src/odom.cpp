#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "SysFsHelper.hpp"

using namespace sysfs;

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  std::string pos_left_file = resolveFilePath("/sys/devices/ocp.*/48304000.epwmss/48304180.eqep/position");
  std::string pos_right_file = resolveFilePath("/sys/devices/ocp.*/48302000.epwmss/48302180.eqep/position");

  double x = 0.0f;
  double y = 0.0f;
  double th = 0.0f;

  double vx = 0.0f;
  double vy = 0.0f;
  double vth = 0.0f;

  const double coef_left = 1.0 / 336.0;
  const double coef_right = 1.0 / 406.0;
  double coef_angle = 3.712756079;
  int left_last = readFile<int>(pos_left_file);
  int right_last = readFile<int>(pos_right_file);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(22.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec(); // Time delta
    int left = readFile<int>(pos_left_file);
    int dleft = left - left_last; // left tick delta
    left_last = left;
    double dleft_meter = ((double)dleft) * coef_left; // left meters delta
    int right = readFile<int>(pos_right_file);
    int dright = right - right_last;
    right_last = right;
    double dright_meter = ((double)dright) * coef_right;
    vx = ((double) (dright_meter + dleft_meter)) / (2.0 * dt);
    vth = ((double)(dright_meter - dleft_meter)) * coef_angle / dt;
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    ROS_INFO("x:%f y:%f th:%f", x, y, th);
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}

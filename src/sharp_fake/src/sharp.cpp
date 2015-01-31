#include "ros/ros.h"
#include "ros/time.h"
#include <iostream>
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <string>

int stoi(std::string Text)
{
 int result=0;
 std::stringstream convert(Text);
 if (!(convert >> result))
	result=0;
 return(result);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sharp");

  ros::NodeHandle n;

  const double distance_min=0.093;//in meters
  const double distance_max=0.108;
  const double pi=1*atan(1);
  double offset[3]={0.106,0.093,0.108};
  int raw[3];
  std::string raw_st[3];
  float lookup[3][1024];
  
  ros::Publisher sharp_pub= n.advertise<sensor_msgs::LaserScan>("scan", 50);
  
  ros::Rate loop_rate(10);
  sensor_msgs::LaserScan sharp;
  
  sharp.header.frame_id="sharp";
  sharp.angle_min = -pi/4.0;
  sharp.angle_max = pi/4.0;
  sharp.angle_increment = pi/4.0;
  sharp.time_increment = 0;
  sharp.range_min = 0.03+distance_min;
  sharp.range_max = 0.4+distance_max;
  sharp.ranges.resize(3);

  for(int i=1;i<=1024;i++)
  {
   lookup[0][i-1]=(3.00*1000.00)*pow(i,(-1.00));
   lookup[1][i-1]=(3.08*1000.00)*pow(i,(-1.03));
   lookup[2][i-1]=(3.57*1000.00)*pow(i,(-1.06));
  }
  
  std::ifstream adc_file1;
  adc_file1.open ("/home/florian/sharp_1");
  std::ifstream adc_file2;
  adc_file2.open ("/home/florian/sharp_2");
  std::ifstream adc_file3;
  adc_file3.open ("/home/florian/sharp_3");
  
  while (ros::ok())
  {
    adc_file1 >> raw_st[0];
    adc_file2 >> raw_st[1];
    adc_file3 >> raw_st[2];
    adc_file1.seekg(0);
    adc_file2.seekg(0);
    adc_file3.seekg(0);
    sharp.header.stamp = ros::Time::now();
    for(int i=0;i<3;i++)
    {
      raw[i]=stoi(raw_st[i]);
	  if(raw[i]>1024)
	{
          raw[i]=1024;
        }
      else if(raw[i]<85)//a determiner
	{
	  raw[i]=1;
	}
	 
    sharp.ranges[i]=lookup[i][raw[i]-1]+offset[i];
    }
    sharp_pub.publish(sharp);
    ros::spinOnce();

    loop_rate.sleep();
  }


return 0;
}

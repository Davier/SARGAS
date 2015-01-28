#include "ros/ros.h"
#include "ros/time.h"
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <string>

std::string PathToDevice(){
	return("/sys/bus/iio/devices/iio\:device0");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sharp");

  ros::NodeHandle n;

  const int channel1=0;
  const int channel2=1;
  const int channel3=2;
  const double distance_min=0.093;//in meters
  const double distance_max=0.108;
  double offset[3]={0.106,0.093,0.108};
  int raw[3];
  std::string raw_st[3];
  float lookup[3][1024];
  
  ros::Publisher sharp_pub={n.advertise<sensor_msgs::LaserScan>("sharp", 50);
  
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

  for(int i=0;i<1024;i++)
  {
   lookup[0][i]=(1.25*10.00)*pow(i,(-0.47));
   lookup[1][i]=(3.08*10.00)*pow(i,(-1.03));
   lookup[2][i]=(3.57*10.00)*pow(i,(-1.06));
  }

  //configuration de l'ADC
  std::ofstream mode(PathToDevice()+"/mode",std::ios::trunc);//ouverture de fichier mode
  mode << "continuous";

  std::ofstream enable1(PathToDevice()+"/scan_elements/in_voltage"+channel1+"_en",std::ios::trunc);//ouverture de fichier enable
  std::ofstream enable2(PathToDevice()+"/scan_elements/in_voltage"+channel2+"_en",std::ios::trunc);//ouverture de fichier enable
  std::ofstream enable3(PathToDevice()+"/scan_elements/in_voltage"+channel3+"_en",std::ios::trunc);//ouverture de fichier enable
  enable1 << "1";
  enable2 << "1";
  enable3 << "1";
  
  std::ofstream buffer(PathToDevice()+"/buffer/length",std::ios::trunc);
  buffer << "1";
  
  std::ofstream start(PathToDevice()+"/buffer/enable",std::ios::trunc);
  start << "1";
  
  std::ifstream adc_file1 (PathToDevice()+"/in_voltage"+channel1+"_raw");
  std::ifstream adc_file2 (PathToDevice()+"/in_voltage"+channel2+"_raw");
  std::ifstream adc_file3 (PathToDevice()+"/in_voltage"+channel3+"_raw");
  
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
      raw[i]=std::stoi(raw_st[i]);
	  if(raw[i]>1023)
	{
          raw[i]=1023;
        }
      else if(raw[i]<85)//a determiner
	{
	  raw[i]=0;
	}
	 
    sharp.range[i]=lookup[i][raw[i]]+offset[i];
    sharp_pub.publish(sharp);
    }
    ros::spinOnce();

    loop_rate.sleep();
  }


return 0;
}

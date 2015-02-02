#include <ros/ros.h>
#include <ros/time.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <string>

int stoi(std::string Text)
{
 int result=0;
 std::istringstream convert(Text);
 if (!(convert >> result))
	result=0;
 return(result);
}

int main(int argc,char** argv){
	ros::init(argc, argv, "odometry_publisher");
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",50);
	ros::Rate r(1.0);
	std::string raw_st[2];
	double position_actuelle_gauche=0, position_actuelle_droit=0;
	double position_passee_gauche=0, position_passee_droit=0;
	double delta_gauche=0,delta_droit=0;
	double temps_passe=0,interval=0;
	double x=0,y=0,th=0;
	double vx=0,vy=0,vth=0;
	const double pi=4*atan(1);
	const double nb_strip=32;
	const double rayon=69.0/2.0*0.001;//in m
	const double espacement=0.240;//ecart entre les roues en m

	nav_msgs::Odometry odom;
	odom.header.frame_id="odom"; 
	odom.child_frame_id="base_link";
	
	std::ifstream codeur_gauche;
	codeur_gauche.open("/home/florian/odom_1");
	std::ifstream codeur_droit;
	codeur_droit.open("/home/florian/odom_2");
	
	
	while(n.ok())
	{
		odom.header.stamp=ros::Time::now();
		interval=odom.header.stamp.toSec()-temps_passe;
		temps_passe=odom.header.stamp.toSec();
		
		codeur_gauche >> raw_st[0];
		codeur_droit >> raw_st[1];
		
		position_actuelle_gauche=stoi(raw_st[0]);
		position_actuelle_droit=stoi(raw_st[1]);
		
		delta_gauche=(position_actuelle_gauche-position_passee_gauche)*(2.0*pi/nb_strip*rayon);//distance parcourue par la roue gauche en metres apres la derniere mesure
		delta_droit=(position_actuelle_droit-position_passee_droit)*(2.0*pi/nb_strip*rayon);//distance parcourue par la roue gauche en metres apres la derniere mesure
		th=(delta_droit-delta_gauche)/espacement;//calcule l'orientation du robot dans le sens trigonometrique
		x=(delta_droit+delta_gauche)/2;
		vx=x/interval;
		vth=th/interval;

		odom.twist.twist.linear.x=vx;
		odom.twist.twist.linear.y=0;
		odom.twist.twist.angular.z=vth;
		odom.pose.pose.position.x=x;
		odom.pose.pose.position.y=0;
		odom.pose.pose.position.z=0;
		odom.pose.pose.orientation=tf::createQuaternionMsgFromYaw(th);
		odom_pub.publish(odom);

		position_passee_gauche=position_actuelle_gauche;
		position_passee_droit=position_actuelle_droit;

		ros::spinOnce();
		r.sleep();
	}
}

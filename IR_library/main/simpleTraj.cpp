#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <Planar3R.hpp>
	/* Write your code her for publishing to /pubJointStates topic
	** The message type is sensor_msgs/JointState 
	** The name field should be an array of names of all four joints
	** The header.stamp field should be ros::Time::now() 
	** The position field should be an array of double values
	** Keep filling the values inside the while(ros::ok()) loop
	** Elapsed time can be calculated as:
	** ros::Time start = ros::Time::now();
	** double diff = (ros::Time::now() - start).toSec();
	** Make the values sinusodial depending on variable diff or anything you like
	** Publish the msg 
	** The lines to be changed or added are marked*/
int main(int argc, char **argv)
{
	//IRlibrary::Vec2 temp_vector;
	//IRlibrary::Vec2 q;
	//temp_vector << 2,-1;
	ros::init(argc, argv, "genConfig");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);
	
	ros::Duration(0.01).sleep();
	ros::Publisher configPub;
	configPub = n.advertise <sensor_msgs::JointState> ("/pubJointStates", 5); /* Fix this line. Do NOT change "/pubJointStates" */
	ros::Time start = ros::Time::now();
	sensor_msgs::JointState new_state;
	new_state.name = {"joint1", "joint2","joint3"};
	new_state.header.stamp = ros::Time::now();
	double diff = (ros::Time::now() - start).toSec();
	new_state.position = { M_PI * cos(diff), M_PI * sin(diff)};
	double l1, l2,l3;
	n.getParam("link_lengths/l1", l1);
	n.getParam("link_lengths/l2", l2);
        n.getParam("link_lengths/l2", l3);
	IRlibrary::Planar3R obj3r;
	obj3r.setLinks(l1,l2,l3);
	
	while (ros::ok())
	{	
		diff = (ros::Time::now() - start).toSec();
		double midRad = ((l1+l2)+std::fabs(l1-l2))/2.;
		double ang = diff/2.;
		
		IRlibrary::Vec2 xy;
		xy << midRad * cos(ang), midRad * sin(ang);
		obj3r.setXY(xy);
		auto q = obj3r.getConfig();
		obj3r.setConfig(q);
		auto xy1 = obj2r.getXY();
		double error_normSquared = (xy - xy1).squaredNorm();
		
		new_state.header.stamp = ros::Time::now();
		new_state.position[0] = q[0];
		new_state.position[1] = q[1];
		configPub.publish(new_state);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

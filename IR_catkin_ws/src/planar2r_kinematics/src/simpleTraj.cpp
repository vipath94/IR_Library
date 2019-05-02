#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <Planar2R.hpp>
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
	ros::init(argc, argv, "genConfig");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);
	
	ros::Duration(0.01).sleep();
	ros::Publisher configPub;
	configPub = n.advertise <sensor_msgs::JointState> ("/pubJointStates", 5); /* Fix this line. Do NOT change "/pubJointStates" */
	ros::Time start = ros::Time::now();
	sensor_msgs::JointState new_state;
	new_state.name = {"joint1", "joint2"};
	new_state.header.stamp = ros::Time::now();
	double diff = (ros::Time::now() - start).toSec();
	new_state.position = { M_PI * cos(diff), M_PI * sin(diff)};
	double l1, l2;
	n.getParam("link_lengths/l1", l1);
	n.getParam("link_lengths/l2", l2);
	IRlibrary::Planar2R obj2r;
	obj2r.setLinks(l1,l2);

    double array[25][2];
        array[0][0] = 0.0; array[0][1] = 1.5;
        array[1][0] = 0.0; array[1][1] = 1.75;
        array[2][0] = 0.0; array[2][1] = 2.0;

        array[3][0] = 0.0; array[3][1] = 2.25;
        array[4][0] = 0.0; array[4][1] = 2.5;
        array[5][0] = 0.0; array[5][1] = 2.75;
        array[6][0] = 0.0; array[6][1] = 3.0;

        array[7][0] = 0.25; array[7][1] = 3.0;
        array[8][0] = 0.5; array[8][1] = 3.0;
        array[9][0] = 0.75; array[9][1] = 3.0;
        array[10][0] = 1.0; array[10][1] = 3.0;

        array[11][0] = 1.25; array[11][1] = 3.0;
        array[12][0] = 1.5; array[12][1] = 3.0;
        array[13][0] = 1.75; array[13][1] = 3.0;
        array[14][0] = 2.0; array[14][1] = 3.0;
       
        array[15][0] = 2.25; array[15][1] = 3.0;
        array[16][0] = 2.5; array[16][1] = 3.0;
        array[17][0] = 2.75; array[17][1] = 3.0;
        array[18][0] = 3.0; array[18][1] = 3.0;

    
    for (int i=0;i<20&&ros::ok();i++)
    {	
		IRlibrary::Vec2 xy;
		xy << array[i][0] , array[i][1] ;
		IRlibrary::Planar2R obj2r;
		obj2r.setLinks(l1, l2);
		obj2r.setXY(xy);
		auto q = obj2r.getConfig();
		new_state.position[0] = q[0];
		new_state.position[1] = q[1];
		configPub.publish(new_state); ros::Duration(1.0).sleep();
    }
	
	return 0;
}

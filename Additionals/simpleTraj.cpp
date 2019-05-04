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
	ros::init(argc, argv, "simpleTraj");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);
	
	ros::Duration(0.01).sleep();
	ros::Publisher configPub;
	configPub = n.advertise <sensor_msgs::JointState> ("/pubJointStates", 5); /* Fix this line. Do NOT change "/pubJointStates" */
	ros::Time start = ros::Time::now();
	sensor_msgs::JointState new_state;
	new_state.name = {"joint1", "joint2","joint3"};
	new_state.header.stamp = ros::Time::now();

	new_state.position = { 0, 0, 0};

	double l1, l2,l3;
	n.getParam("link_lengths/l1", l1);
	n.getParam("link_lengths/l2", l2);
        n.getParam("link_lengths/l3", l3);

	IRlibrary::Planar3R obj3r;
	obj3r.setLinks(l1,l2,l3);
	
     double array[25][3];
        array[0][0] = 0.0; array[0][1] = 4.5; array[0][2] = M_PI/2;
        array[1][0] = 0.0; array[1][1] = 4.75;array[1][2] = M_PI/2;
        array[2][0] = 0.0; array[2][1] = 4.0;array[2][2] = M_PI/2;

        array[3][0] = 0.0; array[3][1] = 5.25;array[3][2] = M_PI/2;
        array[4][0] = 0.0; array[4][1] = 5.5;array[4][2] = M_PI/2;
        array[5][0] = 0.0; array[5][1] = 5.75;array[5][2] = M_PI/2;
        array[6][0] = 0.0; array[6][1] = 5.0;array[6][2] = M_PI/2;

        array[7][0] = 0.25; array[7][1] = 5.0;array[7][2] = M_PI/2;
        array[8][0] = 0.5; array[8][1] = 5.0;array[8][2] = M_PI/2;
        array[9][0] = 0.75; array[9][1] = 5.0;array[9][2] = M_PI/2;
        array[10][0] = 1.0; array[10][1] = 5.0;array[10][2] = M_PI/2;

        array[11][0] = 1.25; array[11][1] = 5.0;array[11][2] = M_PI/2;
        array[12][0] = 1.5; array[12][1] = 5.0;array[12][2] = M_PI/2;
        array[13][0] = 1.75; array[13][1] = 5.0;array[13][2] = M_PI/2;
        array[14][0] = 2.0; array[14][1] = 5.0;array[14][2] = M_PI/2;
       
        array[15][0] = 2.25; array[15][1] = 5.0;array[15][2] = M_PI/2;
        array[16][0] = 2.5; array[16][1] = 5.0;array[16][2] = M_PI/2;
        array[17][0] = 2.75; array[17][1] = 5.0;array[17][2] = M_PI/2;
        array[18][0] = 3.0; array[18][1] = 5.0;array[18][2] = M_PI/2;

    
    for (int i=0;i<19&&ros::ok();i++)
    {	
		IRlibrary::Vec3 xy;
		xy << array[i][0] , array[i][1], array[i][2] ;
		IRlibrary::Planar3R obj3r;
		obj3r.setLinks(l1, l2, l3);
		obj3r.setX(xy);
                IRlibrary::Vec3 init_thetalist = xy;
                init_thetalist[0] -= M_PI/180.; 
                init_thetalist[1] -= M_PI/180.;
                init_thetalist[2] -= M_PI/180.;
                obj3r.inverseKinematics_numerical(init_thetalist,xy);
                auto q = obj3r.getConfig();
		new_state.position[0] = q[0];
		new_state.position[1] = q[1];
                new_state.position[2] = q[2];
		configPub.publish(new_state); ros::Duration(1.0).sleep();
    }
	
	return 0;
}

#include <ros/ros.h>
#include "Odom_Drift.h"

int main(int argc, char *argv[])
{
	// initialization
	ros::init(argc, argv, "Odom_Drift");

	ros::NodeHandle nHandle("~");


	Odom_Drift *od = new Odom_Drift(nHandle);

	// return the control to ROS
	ros::spin();

	return 0;
}

/*
 * Odom_Drift.h
 *
 *  Created on: Jun 8, 2016
 *      Author: andy
 */
/*
 * Odom_Drift.h
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */

#ifndef SRC_Odom_Drift_H_
#define SRC_Odom_Drift_H_

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"


#include <iostream>
#include <vector>


using namespace std;

class Odom_Drift{
public:

	ros::Subscriber loc_sub;
	ros::Publisher loc_pub, path_pub, path_drift_pub;

	void locationCallback( const nav_msgs::Odometry& locIn );
	
	void publish_loc(const nav_msgs::Odometry& loc_in);
	void publish_path(const nav_msgs::Path& path_in);

	nav_msgs::Path path_clean, path_drift;
	nav_msgs::Odometry odom_drift, odom_km1;
	
	double speed_mean, speed_var, perp_mean, perp_var, yaw_mean, yaw_var;
	double x_z_mean, x_z_var, y_z_mean, y_z_var, yaw_z_mean, yaw_z_var;

	bool init;

	// Odom_Drift stuff
	Odom_Drift(ros::NodeHandle nh);
	~Odom_Drift();
};

#endif /* SRC_Odom_Drift_H_ */

/*
 * Odom_Drift.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */


#include "Odom_Drift.h"

#include "geometry_msgs/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf/transform_datatypes.h"
#include <random>
#include <chrono>

Odom_Drift::Odom_Drift(ros::NodeHandle nHandle){
	this->loc_sub = nHandle.subscribe("/odom", 1, &Odom_Drift::locationCallback, this);
	
	this->loc_pub = nHandle.advertise<nav_msgs::Odometry>("/odom_drift", 10);
	this->path_pub = nHandle.advertise<nav_msgs::Path>("/path",10);
	this->path_drift_pub = nHandle.advertise<nav_msgs::Path>("/drift_path",10);

	double drift;
	if (nHandle.getParam("/drift_speed_mean", drift)){
		this->speed_mean = drift;
	}
	else{
		this->speed_mean = 0.0;
	}
	if (nHandle.getParam("/drift_perp_mean", drift)){
		this->perp_mean = drift;
	}
	else{
		this->perp_mean = 0.0;
	}
	if (nHandle.getParam("/drift_yaw_mean", drift)){
		this->yaw_mean = drift;
	}
	else{
		this->yaw_mean = 0.0;
	}
	if (nHandle.getParam("/drift_speed_var", drift)){
		this->speed_var = drift;
	}
	else{
		this->speed_var = 0.0;
	}
	if (nHandle.getParam("/drift_perp_var", drift)){
		this->perp_var = drift;
	}
	else{
		this->perp_var = 0.0;
	}
	if (nHandle.getParam("/drift_yaw_var", drift)){
		this->yaw_var = drift;
	}
	else{
		this->yaw_var = 0.0;
	}
	if (nHandle.getParam("/drift_x_start_mean", drift)){
		this->x_z_mean = drift;
	}
	else{
		this->x_z_mean = 1.0;
	}
	if (nHandle.getParam("/drift_y_start_mean", drift)){
		this->y_z_mean = drift;
	}
	else{
		this->y_z_mean = 1.0;
	}
	if (nHandle.getParam("/drift_yaw_start_mean", drift)){
		this->yaw_z_mean = drift;
	}
	else{
		this->yaw_z_mean = 0.01;
	}
	if (nHandle.getParam("/drift_x_start_var", drift)){
		this->x_z_var = drift;
	}
	else{
		this->x_z_var = 0.01;
	}
	if (nHandle.getParam("/drift_y_start_var", drift)){
		this->y_z_var = drift;
	}
	else{
		this->y_z_var = 0.01;
	}
	if (nHandle.getParam("/drift_yaw_start_var", drift)){
		this->yaw_z_var = drift;
	}
	else{
		this->yaw_z_var = 0.01;
	}

	this->init = false;
}

void Odom_Drift::locationCallback( const nav_msgs::Odometry& odom_in ){

	if(this->init == false){
		this->odom_drift = odom_in;
		this->odom_km1 = odom_in;

		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		std::default_random_engine generator(seed);
	  	std::normal_distribution<double> distribution_x(this->x_z_mean,this->x_z_var);
  		std::normal_distribution<double> distribution_y(this->y_z_mean,this->y_z_var);
		this->odom_drift.pose.pose.position.x += distribution_x(generator);
		this->odom_drift.pose.pose.position.y += distribution_y(generator);

		std::normal_distribution<double> distribution_yaw(this->yaw_z_mean,this->yaw_z_var);
		// the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    	tf::Quaternion quat;
    	tf::quaternionMsgToTF(odom_in.pose.pose.orientation, quat);

    	// the tf::Quaternion has a method to acess roll pitch and yaw
    	double roll, pitch, yaw;
    	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		yaw += distribution_yaw(generator);
		tf::Quaternion q;
		q.setRPY( roll, pitch, yaw );
		q.normalize();
		tf::quaternionTFToMsg(q, this->odom_drift.pose.pose.orientation);

		geometry_msgs::PoseStamped p;
		p.header = odom_in.header;
		p.pose.position = odom_in.pose.pose.position;
		p.pose.orientation = odom_in.pose.pose.orientation;
		this->path_clean.poses.push_back(p);

		p.pose.position = odom_drift.pose.pose.position;
		p.pose.orientation = odom_drift.pose.pose.orientation;
		this->path_drift.poses.push_back(p);
		
		this->path_clean.header = odom_in.header;
		this->path_drift.header = odom_in.header;

		this->loc_pub.publish(odom_drift);
		this->path_pub.publish(this->path_clean);
		this->path_drift_pub.publish(this->path_drift);

		this->init = true;
	}
	else{
		double max_dist = 1.0;
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		std::default_random_engine generator(seed);
		
		// how far did I actually move in the last time step, add noise to distance
		double dx = odom_in.pose.pose.position.x - this->odom_km1.pose.pose.position.x;
		double dy = odom_in.pose.pose.position.y - this->odom_km1.pose.pose.position.y;
		std::normal_distribution<double> distribution_speed(this->speed_mean,this->speed_var);
		double dist = sqrt(pow(dx,2)+pow(dy,2));
		dist += distribution_speed(generator);
		std::normal_distribution<double> distribution_perp(this->perp_mean,this->perp_var);
		double perp = distribution_speed(generator);

		// get heading of previous timestep, add noise to heading
    	tf::Quaternion quat; // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    	tf::quaternionMsgToTF(odom_in.pose.pose.orientation, quat); // the tf::Quaternion has a method to acess roll pitch and yaw
    	double roll, pitch, yaw_in;
    	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw_in);
    	// get heading of drift
    	tf::quaternionMsgToTF(odom_drift.pose.pose.orientation, quat); // the tf::Quaternion has a method to acess roll pitch and yaw
    	double yaw_drift;
    	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw_drift);
    	// get diff between actual and drift headings
    	double yaw_error = yaw_in - yaw_drift;
    	// this acts as a soft constraint on drift heading, prevents it from growing too large
		std::normal_distribution<double> distribution_yaw(this->yaw_mean + yaw_error,this->yaw_var);
		// update current drift heading
		yaw_drift += distribution_yaw(generator);

		tf::Quaternion q;
		q.setRPY( roll, pitch, yaw_drift );
		q.normalize();

		// update drift position
		this->odom_drift.pose.pose.position.x += dist * cos(yaw_drift) + perp * -sin(yaw_drift);
		this->odom_drift.pose.pose.position.y += dist * sin(yaw_drift) + perp * cos(yaw_drift);
		tf::quaternionTFToMsg(q, this->odom_drift.pose.pose.orientation);
		
		// convert to PoseStamped for path
		geometry_msgs::PoseStamped p;
		p.header = odom_in.header;
		p.pose.position = odom_in.pose.pose.position;
		p.pose.orientation = odom_in.pose.pose.orientation;
		this->path_clean.poses.push_back(p);
		p.pose.position = this->odom_drift.pose.pose.position;
		p.pose.orientation = this->odom_drift.pose.pose.orientation;
		this->path_drift.poses.push_back(p);

		// publish odom and paths
		this->loc_pub.publish(this->odom_drift);
		this->path_pub.publish(this->path_clean);
		this->path_drift_pub.publish(this->path_drift);
		// update odom km1 to get next distance
		this->odom_km1 = odom_in;
	}
}






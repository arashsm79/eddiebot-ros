#include "eddiebot_virtual_driver.h"

#define PI 				3.14159265359
#define TWOPI			((PI) * 2)
#define LINEAR_STEP		0.05
#define ANGULAR_STEP	(10 / (PI))

EddieBotVirtualDriver::EddieBotVirtualDriver()
{
	x_ = y_ = th_ = 0;
	vx_ = vy_ = vth_ = 0;
	last_time_ = ros::Time::now();
	
	sem_init(&mutex_data_, 0, 1);

	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
	cmd_vel_sub_ = nh_.subscribe("cmd_vel", 5, 
		&EddieBotVirtualDriver::cmd_vel_cb_, this);
}

void EddieBotVirtualDriver::cmd_vel_cb_(const eddiebot_msgs::Velocity::ConstPtr &msg)
{
	
	double linear, angular;

	if(msg->linear == 0)
		linear = 0;
	else
		linear = msg->linear > 0 ? LINEAR_STEP : -LINEAR_STEP;

	if(msg->angular == 0)
		angular = 0;
	else
		angular = msg->angular > 0 ? LINEAR_STEP : -LINEAR_STEP;


	sem_wait(&mutex_data_);

	// accumulate total rotation around our center
	th_ += angular;
	// clip the rotation to plus or minus 360 degrees
	th_ -= (double)((int)(th_ / TWOPI)) * TWOPI;

	vx_ = linear * sin(th_);
	vy_ = linear * cos(th_);
	vth_ = angular;

	x_ += vx_;
	y_ += vy_;

	sem_post(&mutex_data_);
}

void EddieBotVirtualDriver::publish_odom()
{
	double x, y, th;
	double vx, vy, vth;
	ros::Time current_time;

	ros::Rate r(1.0);

	while(nh_.ok()){
		ros::spinOnce();

		sem_wait(&mutex_data_);

		x = x_;
		y = y_;
		th = th_;
		vx = vx_;
		vy = vy_;
		vth = vth_;

		// let cmd_vel_cb_ update velocity
		vx_ = vy_ = vth_ = 0;

		sem_post(&mutex_data_);

		current_time = ros::Time::now();

		//since all odometry is 6DOF we'll need a quaternion created from yaw
	    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	    //first, we'll publish the transform over tf
	    geometry_msgs::TransformStamped odom_trans;
	    odom_trans.header.stamp = current_time;
	    odom_trans.header.frame_id = "odom";
	    odom_trans.child_frame_id = "base_footprint";
	    odom_trans.transform.translation.x = x;
	    odom_trans.transform.translation.y = y;
	    odom_trans.transform.translation.z = 0.0;
	    odom_trans.transform.rotation = odom_quat;
	    //send the transform
	    odom_broadcaster_.sendTransform(odom_trans);

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
	    odom.child_frame_id = "base_footprint";
	    odom.twist.twist.linear.x = vx;
	    odom.twist.twist.linear.y = vy;
	    odom.twist.twist.angular.z = vth; 
	    
	    //publish the message
	    odom_pub_.publish(odom);

	    sem_wait(&mutex_data_);
	    last_time_ = current_time;
	    sem_post(&mutex_data_);

	    r.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "eddiebot_virtual_driver");
	EddieBotVirtualDriver eddiebot;

	eddiebot.publish_odom();

	return 0;
}
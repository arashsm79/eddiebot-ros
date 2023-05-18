#include "eddiebot_virtual_driver/eddiebot_virtual_driver.h"


#define PI 				3.14159265359
#define TWOPI			((PI) * 2)
#define LINEAR_STEP		0.05
#define ANGULAR_STEP	(10 / (PI))

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

EddieBotVirtualDriver::EddieBotVirtualDriver(std::shared_ptr<rclcpp::Node> node_handle) : nh_(node_handle)
{
	x_ = y_ = th_ = 0;
	vx_ = vy_ = vth_ = 0;
	last_time_ = nh_->get_clock()->now();
	
	sem_init(&mutex_data_, 0, 1);

	odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
	cmd_vel_sub_ = nh_->create_subscription<eddiebot_msgs::msg::Velocity>("cmd_vel", 5,
      std::bind(&EddieBotVirtualDriver::cmd_vel_cb_, this, std::placeholders::_1));
}

void EddieBotVirtualDriver::cmd_vel_cb_(const eddiebot_msgs::msg::Velocity::ConstSharedPtr msg)
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
	rclcpp::Time current_time;

	rclcpp::Rate r(1.0);

	while(rclcpp::ok()){
		rclcpp::spin_some(nh_);

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

		current_time = nh_->get_clock()->now();

		//since all odometry is 6DOF we'll need a quaternion created from yaw
	    geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(th);
	    //first, we'll publish the transform over tf
	    geometry_msgs::msg::TransformStamped odom_trans;
	    odom_trans.header.stamp = current_time;
	    odom_trans.header.frame_id = "odom";
	    odom_trans.child_frame_id = "base_footprint";
	    odom_trans.transform.translation.x = x;
	    odom_trans.transform.translation.y = y;
	    odom_trans.transform.translation.z = 0.0;
	    odom_trans.transform.rotation = odom_quat;
	    //send the transform
	    odom_broadcaster_->sendTransform(odom_trans);

	    //next, we'll publish the odometry message over ROS
	    nav_msgs::msg::Odometry odom;
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
	    odom_pub_->publish(odom);

	    sem_wait(&mutex_data_);
	    last_time_ = current_time;
	    sem_post(&mutex_data_);

	    r.sleep();
	}
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node_handle = rclcpp::Node::make_shared("eddiebot_virtual_driver");
	EddieBotVirtualDriver eddiebot(node_handle);

	eddiebot.publish_odom();

	return 0;
}

#include "eddiebot_odom/eddie_odom.h"

using namespace std;

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

EddieOdomPublisher::EddieOdomPublisher(std::shared_ptr<rclcpp::Node> node_handle) : nh_(node_handle)
{
    odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    encoders_sub_ = nh_->create_subscription<eddiebot_msgs::msg::Encoders>("/eddie/encoders_data", 1,
      std::bind(&EddieOdomPublisher::encoder_cb_, this, std::placeholders::_1));

    x_ = y_ = th_ = 0.0;
    
    prev_left_encoder_cnt_ = prev_right_encoder_cnt_ = 0;

    current_time_ = last_time_ = nh_->get_clock()->now();
}

void EddieOdomPublisher::encoder_cb_(const eddiebot_msgs::msg::Encoders::ConstSharedPtr msg)
{
    current_time_ = nh_->get_clock()->now();
    double dt = (current_time_ - last_time_).seconds();

    // msg->left(right) is to the total tick of the left(right) encoder
    // delta_left_cnt represents the increment of the left encoder ticks
    int delta_left_cnt = msg->left - prev_left_encoder_cnt_;
    int delta_right_cnt = msg->right - prev_right_encoder_cnt_;

    double delta_th = 1.0 * (delta_right_cnt - delta_left_cnt) * DISTANCE_PER_COUNT / WHEEL_BASE;
    double delta_dist = 0.5 * (delta_right_cnt + delta_left_cnt) * DISTANCE_PER_COUNT;
    double delta_x = delta_dist * cos(th_);
    double delta_y = delta_dist * sin(th_);

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    if(th_ > TWOPI)
        th_ -= TWOPI;
    else if(th_ <= -TWOPI)
        th_ += TWOPI;

    // printf("x = %lf, y = %lf, th = %lf\n", x_, y_, th_);

    prev_left_encoder_cnt_ = msg->left;
    prev_right_encoder_cnt_ = msg->right;
    last_time_ = current_time_;

    publish_odom_(delta_x, delta_y, delta_th, dt);
}

void EddieOdomPublisher::publish_odom_(double dx, double dy, double dth, double dt)
{
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(th_);
    //first, we'll publish the transform over tf
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    odom_broadcaster_->sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = dx / dt;
    odom.twist.twist.linear.y = dy / dt;
    odom.twist.twist.angular.z = dth / dt; 
    
    //publish the message
    odom_pub_->publish(odom);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node_handle = rclcpp::Node::make_shared("eddie_odom");
  EddieOdomPublisher eddie_odom_pub(node_handle);

  rclcpp::spin(node_handle);

  return 0;
}


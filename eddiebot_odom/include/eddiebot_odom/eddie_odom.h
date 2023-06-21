#ifndef _EDDIE_ODOM_H
#define _EDDIE_ODOM_H

#include "rclcpp/rclcpp.hpp"
#include "eddiebot_msgs/msg/encoders.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <rclcpp/executors.hpp>
#include <stdio.h>

#define PI      3.14159265359
#define TWOPI   (PI * 2)
// encoder counter per revolution
#define COUNTS_PER_REVOLUTION   36
// Wheel Radius
#define WHEEL_RADIUS    0.1524
// the distance of a wheel move forward when encoder increased by 1
#define DISTANCE_PER_COUNT      ((PI * WHEEL_RADIUS) / COUNTS_PER_REVOLUTION)
// two wheels center-to-center distance
#define WHEEL_BASE      0.39


class EddieOdomPublisher
{
public:
    EddieOdomPublisher(std::shared_ptr<rclcpp::Node>);
    
private:
    void encoder_cb_(const eddiebot_msgs::msg::Encoders::ConstSharedPtr msg);
    void publish_odom_(double dx, double dy, double dth, double dt);

private:
    std::shared_ptr<rclcpp::Node> nh_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<eddiebot_msgs::msg::Encoders>::SharedPtr encoders_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

    // previous position
    double x_;
    double y_;
    double th_;

    // previous encoders count
    int prev_left_encoder_cnt_;
    int prev_right_encoder_cnt_;

    rclcpp::Time current_time_;
    rclcpp::Time last_time_;
};

#endif

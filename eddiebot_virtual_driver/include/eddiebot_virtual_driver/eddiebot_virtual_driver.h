#ifndef _EDDIEBOT_VIRTUAL_DRIVER_H
#define _EDDIEBOT_VIRTUAL_DRIVER_H

#include <semaphore.h>

#include "rclcpp/rclcpp.hpp"
#include "eddiebot_msgs/msg/velocity.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class EddieBotVirtualDriver
{
public:
    EddieBotVirtualDriver(std::shared_ptr<rclcpp::Node>);
    void publish_odom();
    

private:
    void cmd_vel_cb_(const eddiebot_msgs::msg::Velocity::ConstSharedPtr msg);
    

private:
    std::shared_ptr<rclcpp::Node> nh_;
    
    rclcpp::Subscription<eddiebot_msgs::msg::Velocity>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

    sem_t mutex_data_;

    // previous position
    double x_;
    double y_;
    double th_;

    double vx_;
    double vy_;
    double vth_;

    rclcpp::Time last_time_;
};

#endif

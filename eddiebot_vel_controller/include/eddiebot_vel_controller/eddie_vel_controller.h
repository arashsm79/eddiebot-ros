#ifndef _EDDIE_VEL_CONTROLLER_H
#define _EDDIE_VEL_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "eddiebot_msgs/msg/velocity.hpp"
#include "eddiebot_msgs/msg/key_stroke.hpp"
#include "geometry_msgs/msg/twist.hpp"


class EddieVelController
{
public:
    EddieVelController(std::shared_ptr<rclcpp::Node>);
    
private:
	void cmd_vel_callback_(const geometry_msgs::msg::Twist::ConstSharedPtr msg);

private:
    std::shared_ptr<rclcpp::Node> node_handle_;
    rclcpp::Publisher<eddiebot_msgs::msg::Velocity>::SharedPtr vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

#endif

#include "eddiebot_vel_controller/eddie_vel_controller.h"
#include <rclcpp/executors.hpp>

using namespace std;

EddieVelController::EddieVelController(std::shared_ptr<rclcpp::Node> node_handle) : node_handle_(node_handle)
{
    vel_pub_ = node_handle_->create_publisher<eddiebot_msgs::msg::Velocity>("/eddie/cmd_vel", 5);

    cmd_vel_sub_ = node_handle_->create_subscription<geometry_msgs::msg::Twist>(
          "/eddie/raw_cmd_vel", 1,
          std::bind(&EddieVelController::cmd_vel_callback_, this,
                    std::placeholders::_1));
}

void EddieVelController::cmd_vel_callback_(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  eddiebot_msgs::msg::Velocity cmd_vel;

    cmd_vel.linear = msg->linear.x;
    cmd_vel.angular = msg->angular.z;

    vel_pub_->publish(cmd_vel);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_handle = rclcpp::Node::make_shared("eddie_vel_controll");
    EddieVelController eddie_vel_controller(node_handle);

    rclcpp::spin(node_handle);

    return 0;
}

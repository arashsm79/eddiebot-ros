/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Haikal Pribadi <haikal.pribadi@gmail.com>
 * Copyright (c) 2018, Zeyu Zhang <zeyuz@outlook.com>
 * Copyright (c) 2023, Arash Sal Moslehian <arashsm79@yahoo.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the Haikal Pribadi nor the names of other
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _EDDIE_CONTROLLER_H
#define	_EDDIE_CONTROLLER_H

#include <rclcpp/subscription.hpp>
#include <semaphore.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "eddiebot_msgs/msg/distances.hpp"
#include "eddiebot_msgs/msg/velocity.hpp"
#include "eddiebot_msgs/msg/voltages.hpp"
#include "eddiebot_msgs/srv/accelerate.hpp"
#include "eddiebot_msgs/srv/drive_with_distance.hpp"
#include "eddiebot_msgs/srv/drive_with_power.hpp"
#include "eddiebot_msgs/srv/drive_with_speed.hpp"
#include "eddiebot_msgs/srv/get_heading.hpp"
#include "eddiebot_msgs/srv/get_status.hpp"
#include "eddiebot_msgs/srv/reset_encoder.hpp"
#include "eddiebot_msgs/srv/rotate.hpp"
#include "eddiebot_msgs/srv/stop_at_distance.hpp"

#define PI      3.14159265359
#define TWOPI   (PI * 2)
// encoder counter per revolution
#define COUNTS_PER_REVOLUTION   36
// Wheel radius in meters
#define WHEEL_RADIUS    0.1524
// the distance of a wheel move forward when encoder increased by 1
#define DISTANCE_PER_COUNT      ((PI * WHEEL_RADIUS) / COUNTS_PER_REVOLUTION)
// two wheels center-to-center distance
#define WHEEL_BASE      0.39

class EddieController {
public:
    EddieController(std::shared_ptr<rclcpp::Node>);
    void execute();

private:
    std::shared_ptr<rclcpp::Node> node_handle_;
    rclcpp::Subscription<eddiebot_msgs::msg::Velocity>::SharedPtr velocity_sub_;
    rclcpp::Subscription<eddiebot_msgs::msg::Distances>::SharedPtr ping_distances_sub_;
    rclcpp::Subscription<eddiebot_msgs::msg::Voltages>::SharedPtr ir_distances_sub_;
    rclcpp::Service<eddiebot_msgs::srv::GetStatus>::SharedPtr eddie_status_srv_;
    rclcpp::Client<eddiebot_msgs::srv::DriveWithPower>::SharedPtr eddie_drive_power_;
    rclcpp::Client<eddiebot_msgs::srv::DriveWithSpeed>::SharedPtr eddie_drive_speed_;
    rclcpp::Client<eddiebot_msgs::srv::Accelerate>::SharedPtr eddie_acceleration_rate_;
    rclcpp::Client<eddiebot_msgs::srv::Rotate>::SharedPtr eddie_turn_;
    rclcpp::Client<eddiebot_msgs::srv::StopAtDistance>::SharedPtr eddie_stop_;
    rclcpp::Client<eddiebot_msgs::srv::GetHeading>::SharedPtr eddie_heading_;
    rclcpp::Client<eddiebot_msgs::srv::ResetEncoder>::SharedPtr eddie_reset_;

    rclcpp::Time last_cmd_time_;
  
    // Base value for wheel powers. The velocity is multiplied by this value.
    int base_left_power_, base_right_power_, base_rotation_power_;
    int acceleration_power_, deceleration_power_, min_power_;
    int left_speed_, right_speed_, rotation_speed_, acceleration_speed_;
    double linear_scale_, angular_scale_;

    /*
      The values for the left and right wheel that are used in drive()
      The left and right power levels are entered as signed (two’s complement) 8-bit
      hex values. The range of valid values is 81 (full reverse) to 7F (full forward). A
      value entered of 80 will be clipped to 81 to maintain symmetry between positive
      and negative drive power levels.
      This command sets the motor output PWM ratio, which effectively corresponds to the motor
      drive power. The range of values (in decimal terms) is -127 to +127, and corresponds to -
      100% to +100% duty cycle ratio respectively. Note that two’s complement signed 8-bit
      values typically range from -128 to +127; so to maintain symmetry in the effective drive
      power, a value of -128 is clipped to be -127.
    */
    int8_t left_drive_power, right_drive_power;

    // The base power to drive the wheel with. This value is used in what
    // ultimately is sent to the driveWithPower service.
    int8_t current_power_;

    int16_t current_angle_, angular_;

    // True if Eddie only rotates in place
    bool rotate_;

    // Used to cancel the movement of Eddie while running drive() or rotate()
    bool interrupt_;

    // If true, we will issue moving commands such as drive() or rotate()
    // to the robot in the execution loop.
    bool process_;

    bool ping_distances_okay_, ir_distances_okay_;

    sem_t mutex_execute_;
    sem_t mutex_interrupt_;
    sem_t mutex_state_;
    sem_t mutex_ping_;
    sem_t mutex_ir_;

    void velocityCallback(const eddiebot_msgs::msg::Velocity::ConstSharedPtr message);
    void distanceCallback(const eddiebot_msgs::msg::Distances::ConstSharedPtr message);
    void irCallback(const eddiebot_msgs::msg::Voltages::ConstSharedPtr message);
    bool getStatus(eddiebot_msgs::srv::GetStatus::Request::SharedPtr req,
            eddiebot_msgs::srv::GetStatus::Response::SharedPtr res);
    void stop();
    void setAccelerationRate(int rate);
    void moveLinear(float linear);
    void moveAngular(int16_t angular);
    void moveLinearAngular(float linear, int16_t angular);
    void drive(int8_t left, int8_t right);
    void rotate(int16_t angular);
    void updateCurrentPower(int8_t left, int8_t right);
    int8_t clipPower(int power_unit, float linear);
    int16_t clipSpeed(int speed_unit, float linear);
};

#endif	/* _EDDIE_CONTROLLER_H */



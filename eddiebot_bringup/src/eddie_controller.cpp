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

#include "eddiebot_bringup/eddie_controller.h"
#include <rclcpp/executors.hpp>

using namespace std::chrono_literals;

EddieController::EddieController(std::shared_ptr<rclcpp::Node> node_handle)
    : node_handle_(node_handle),  
       acceleration_rate_(30), 
       rotation_speed_(36), min_velocity_(36),
      max_velocity(36), linear_scale_(1.0), angular_scale_(1.0) {

  node_handle_->get_parameter_or("min_velocity", min_velocity_, min_velocity_);
  node_handle_->get_parameter_or("max_velocity", max_velocity, max_velocity);
  node_handle_->get_parameter_or("rotation_speed", rotation_speed_,
                                 rotation_speed_);
  node_handle_->get_parameter_or("acceleration_rate", acceleration_rate_,
                                 acceleration_rate_);
  node_handle_->get_parameter_or("angular_scale", angular_scale_,
                                 angular_scale_);
  node_handle_->get_parameter_or("linear_scale", linear_scale_, linear_scale_);

  RCLCPP_INFO(node_handle_->get_logger(), "Parameters received.");

  sem_init(&mutex_execute_, 0, 1);
  sem_init(&mutex_interrupt_, 0, 1);
  sem_init(&mutex_ping_, 0, 1);
  sem_init(&mutex_ir_, 0, 1);

  sem_wait(&mutex_interrupt_);
  left_drive_speed = 0;
  right_drive_speed = 0;
  angular_drive_speed = 0;
  rotate_ = false;
  process_ = false;
  last_cmd_time_ = node_handle_->get_clock()->now();
  interrupt_ = false;
  sem_post(&mutex_interrupt_);

  RCLCPP_INFO(node_handle_->get_logger(), "Semaphores initiated.");

  velocity_sub_ =
      node_handle_->create_subscription<eddiebot_msgs::msg::Velocity>(
          "/eddie/simple_velocity", 1,
          std::bind(&EddieController::velocityCallback, this,
                    std::placeholders::_1));
  ping_distances_sub_ =
      node_handle_->create_subscription<eddiebot_msgs::msg::Distances>(
          "/eddie/ping_distances", 1,
          std::bind(&EddieController::distanceCallback, this,
                    std::placeholders::_1));
  ir_distances_sub_ =
      node_handle_->create_subscription<eddiebot_msgs::msg::Voltages>(
          "/eddie/ir_voltages", 1,
          std::bind(&EddieController::irCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_handle_->get_logger(), "Subscriptions created.");
  eddie_status_srv_ =
      node_handle_->create_service<eddiebot_msgs::srv::GetStatus>(
          "emergency_status",
          std::bind(&EddieController::getStatus, this, std::placeholders::_1,
                    std::placeholders::_2));
  RCLCPP_INFO(node_handle_->get_logger(), "Service created.");

  eddie_drive_power_ =
      node_handle_->create_client<eddiebot_msgs::srv::DriveWithPower>(
          "drive_with_power");
  eddie_drive_speed_ =
      node_handle_->create_client<eddiebot_msgs::srv::DriveWithSpeed>(
          "drive_with_speed");
  eddie_acceleration_rate_ =
      node_handle_->create_client<eddiebot_msgs::srv::Accelerate>(
          "acceleration_rate");
  eddie_turn_ =
      node_handle_->create_client<eddiebot_msgs::srv::Rotate>("rotate");
  eddie_stop_ = node_handle_->create_client<eddiebot_msgs::srv::StopAtDistance>(
      "stop_at_distance");
  eddie_heading_ = node_handle_->create_client<eddiebot_msgs::srv::GetHeading>(
      "get_heading");
  eddie_reset_ = node_handle_->create_client<eddiebot_msgs::srv::ResetEncoder>(
      "reset_encoder");

  setAccelerationRate(acceleration_rate_);
}

// Receives the velocity command from the velocity topic and
// calls the appropriate function based on the values
// of linear and angular velocities.
void EddieController::velocityCallback(
    const eddiebot_msgs::msg::Velocity::ConstSharedPtr message) {
  float linear = message->linear;
  float angular = message->angular;

  if (linear == 0 && angular == 0) {
    stop();
  } else if (linear != 0 && angular == 0) {
    moveLinear(linear);
  } else if (linear == 0 && angular != 0) {
    moveAngular(angular);
  } else { // if (linear!=0 && angular !=0)
    moveLinearAngular(linear, angular);
  }
}

// If there is an object close by, change the status and stop
void EddieController::distanceCallback(
    const eddiebot_msgs::msg::Distances::ConstSharedPtr message) {
  sem_wait(&mutex_ping_);
  bool okay = true;
  for (uint i = 0; i < message->value.size(); i++) {
    if (message->value[i] != 0 && message->value[i] < 150)
      okay = false;
  }
  ping_distances_okay_ = okay;
  if (!okay)
    stop();
  sem_post(&mutex_ping_);
}

// If there is an object close by, change the status and stop
void EddieController::irCallback(
    const eddiebot_msgs::msg::Voltages::ConstSharedPtr message) {
  sem_wait(&mutex_ir_);
  bool okay = true;
  for (uint i = 0; i < message->value.size(); i++) {
    if (message->value[i] != 0 && message->value[i] > 1.7)
      okay = false;
  }
  ir_distances_okay_ = okay;
  if (!okay)
    stop();
  sem_post(&mutex_ir_);
}

// Status to check if Eddie is not about to contact anything.
bool EddieController::getStatus(
    eddiebot_msgs::srv::GetStatus::Request::SharedPtr req,
    eddiebot_msgs::srv::GetStatus::Response::SharedPtr res) {
  (void)req;

  res->okay = isWithinSafeDistance();

  return true;
}

bool EddieController::isWithinSafeDistance() {
  sem_wait(&mutex_ping_);
  sem_wait(&mutex_ir_);
  bool ret = true;

  if (ping_distances_okay_ && ir_distances_okay_)
    ret = true;
  else
    ret = false;

  sem_post(&mutex_ping_);
  sem_post(&mutex_ir_);
  return ret;
}

// Send a request to stop Eddie
void EddieController::stop() {
  auto dist_req = std::make_shared<eddiebot_msgs::srv::StopAtDistance::Request>();
  dist_req->distance = 0;

  // If Eddie is in the middle of drive() or rotate()
  // cancel its movement
  sem_wait(&mutex_interrupt_);
  interrupt_ = true;
  sem_post(&mutex_interrupt_);

  while (!eddie_stop_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_handle_->get_logger(), "Interrupted while waiting for the EddiebotStop service. Exiting.");
      return;
    }
    RCLCPP_INFO(node_handle_->get_logger(), "EddiebotStop Service not available, waiting again...");
  }
  auto result = eddie_stop_->async_send_request(dist_req);
}

void EddieController::setAccelerationRate(int rate) {
  auto acc_req = std::make_shared<eddiebot_msgs::srv::Accelerate::Request>();

  acc_req->rate = rate;

  while (!eddie_acceleration_rate_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_handle_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node_handle_->get_logger(), "Service not available, waiting again...");
  }
  auto result = eddie_acceleration_rate_->async_send_request(acc_req);
  if (rclcpp::spin_until_future_complete(node_handle_, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node_handle_->get_logger(), "Sent Eddie acceleration rate request to service.");
  } else {
    RCLCPP_ERROR(node_handle_->get_logger(), "ERROR: Failed to set acceleration rate to %d", rate);
  }
}

// Called from the velocity callback.
// Set the values for the left and right wheels' speeds
// so that Eddie moves in a straight line.
void EddieController::moveLinear(float linear) {

  // Calculate wheel velocities and convert meter per second to position per second
  double left_command_speed  = (linear / WHEEL_RADIUS) / DISTANCE_PER_COUNT;
  double right_command_speed = (linear / WHEEL_RADIUS) / DISTANCE_PER_COUNT;

  sem_wait(&mutex_interrupt_);
  left_drive_speed = left_command_speed;
  right_drive_speed = right_command_speed;
  // this is not a pure rotation
  rotate_ = false;
  process_ = true;
  // cancel other moving commands in favor of this new one.
  interrupt_ = true;
  sem_post(&mutex_interrupt_);
}

// Called from the velocity callback.
// Rotate the robot in place
void EddieController::moveAngular(float angular) {

  // Calculate angular velocities and convert meter per second to position per second
  int16_t angular_command_speed = ((angular * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS) / DISTANCE_PER_COUNT;

  sem_wait(&mutex_interrupt_);
  left_drive_speed = 0;
  right_drive_speed = 0;
  angular_drive_speed = angular_command_speed;
  rotate_ = true;
  process_ = true;
  // cancel other moving commands in favor of this new one.
  interrupt_ = true;
  sem_post(&mutex_interrupt_);
}

// Called from the velocity callback.
// Set the values for the left and right wheels' speeds
// so that Eddie can do arcs
void EddieController::moveLinearAngular(float linear, float angular) {

  // Calculate wheel velocities and convert meter per second to position per second
  double left_command_speed  = ((linear - angular * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS) / DISTANCE_PER_COUNT;
  double right_command_speed = ((linear + angular * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS) / DISTANCE_PER_COUNT;

  sem_wait(&mutex_interrupt_);
  left_drive_speed  = left_command_speed;
  right_drive_speed = right_command_speed;
  // this is not a pure rotation
  rotate_ = false;
  process_ = true;
  // cancel other moving commands in favor of this new one.
  interrupt_ = true;
  sem_post(&mutex_interrupt_);
}

// Send request to speed srv passing in the velocities of each wheel
void EddieController::drive(int16_t left, int16_t right) {
  sem_wait(&mutex_execute_);

  auto drive_with_speed_req = std::make_shared<eddiebot_msgs::srv::DriveWithSpeed::Request>();

  rclcpp::Time now;

  drive_with_speed_req->left  = clipSpeed(left);
  drive_with_speed_req->right = clipSpeed(right);

  RCLCPP_INFO(node_handle_->get_logger(), "wheel vel: left = %d, right = %d", drive_with_speed_req->left, drive_with_speed_req->right);

  while (!eddie_drive_speed_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_handle_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node_handle_->get_logger(), "Service not available, waiting again...");
  }
  auto result = eddie_drive_speed_->async_send_request(drive_with_speed_req);
  if (rclcpp::spin_until_future_complete(node_handle_, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node_handle_->get_logger(), "Sent Eddie drive power request to service.");
  } else {
    RCLCPP_ERROR(node_handle_->get_logger(), "ERROR: at trying to drive Eddie.");
  }

  sem_post(&mutex_execute_);
}

// Rotate Eddie in place.
void EddieController::rotate(int16_t angular) {
  sem_wait(&mutex_execute_);

  auto rotate_req = std::make_shared<eddiebot_msgs::srv::Rotate::Request>();

  rclcpp::Time now;

  rotate_req->angle  = angular > 0 ? 32000 : -32000;
  rotate_req->speed  = clipAngularSpeed(angular);

  RCLCPP_INFO(node_handle_->get_logger(), "angular: %d, dist: %lf, %lf", angular, DISTANCE_PER_COUNT, angular / DISTANCE_PER_COUNT);
  RCLCPP_INFO(node_handle_->get_logger(), "wheel angular speed: angle = %d, speed = %d", rotate_req->angle, rotate_req->speed);

  while (!eddie_turn_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_handle_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node_handle_->get_logger(), "Service not available, waiting again...");
  }
  auto result = eddie_turn_->async_send_request(rotate_req);
  if (rclcpp::spin_until_future_complete(node_handle_, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node_handle_->get_logger(), "Sent Eddie drive power request to service.");
  } else {
    RCLCPP_ERROR(node_handle_->get_logger(), "ERROR: at trying to drive Eddie.");
  }

  sem_post(&mutex_execute_);
}

// Make sure the power value is between -127 (0x80) and 127 (0x7F)
int8_t EddieController::clipPower(int power_unit, float linear) {
  int8_t power;

  if (power_unit * abs(linear) > 127)
    power = linear > 0 ? 127 : -127;
  else
    power = power_unit * linear;

  return power;
}

// Make sure the power value is between 32767 (0x8000) and -32767 (0x7FFF)
int16_t EddieController::clipSpeed(int32_t speed_command) {
  int16_t speed;

  if (speed_command  > 32767)
    speed = 32767;
  else if (speed_command < -32767)
    speed = -32767;
  else
    speed = speed_command;

  return speed;
}

// Make sure the power value is between 65536 (0xFFFF) and 0
int16_t EddieController::clipAngularSpeed(int32_t angular_speed_command) {
  uint16_t speed;

  int32_t abs_angular_speed_command = abs(angular_speed_command);

  if (abs_angular_speed_command  > 65535)
    speed = 65535;
  else
    speed = abs_angular_speed_command;

  return speed;
}

// Main execution loop
void EddieController::execute() {
  rclcpp::Rate rate(1000);
  while (rclcpp::ok()) {
    sem_wait(&mutex_interrupt_);
    bool ex = process_;
    sem_post(&mutex_interrupt_);
    if (ex) {
      sem_wait(&mutex_interrupt_);
      process_ = false;
      int16_t l = left_drive_speed;
      int16_t r = right_drive_speed;
      bool rot = rotate_;
      int16_t angular = angular_drive_speed;
      sem_post(&mutex_interrupt_);
      if (rot)
        rotate(angular);
      else
        drive(l, r);
    }

    rclcpp::spin_some(node_handle_);
    rate.sleep();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node_handle = rclcpp::Node::make_shared("eddie_controller");
  RCLCPP_INFO(node_handle->get_logger(), "eddie_controller created.");
  EddieController controller(node_handle);
  controller.execute();

  return (EXIT_SUCCESS);
}

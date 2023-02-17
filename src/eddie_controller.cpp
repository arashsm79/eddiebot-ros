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

#include "eddie_controller.h"

EddieController::EddieController() :
  left_power_(60), right_power_(62), rotation_power_(40),
  acceleration_power_(30), deceleration_power_(100), min_power_(32),
  left_speed_(36), right_speed_(36), rotation_speed_(36), acceleration_speed_(36),
  linear_scale_(1.0), angular_scale_(1.0)
{
  node_handle_.param("left_power", left_power_, left_power_);
  node_handle_.param("right_power", right_power_, right_power_);
  node_handle_.param("rotation_power", rotation_power_, rotation_power_);
  node_handle_.param("acceleration_power", acceleration_power_, acceleration_power_);
  node_handle_.param("deceleration_power", deceleration_power_, deceleration_power_);
  node_handle_.param("min_power", min_power_, min_power_);
  node_handle_.param("left_speed", left_speed_, left_speed_);
  node_handle_.param("right_speed", right_speed_, right_speed_);
  node_handle_.param("rotation_speed", rotation_speed_, rotation_speed_);
  node_handle_.param("acceleration_speed", acceleration_speed_, acceleration_speed_);

  node_handle_.param("angular_scale", angular_scale_, angular_scale_);
  node_handle_.param("linear_scale", linear_scale_, linear_scale_);

  sem_init(&mutex_execute_, 0, 1);
  sem_init(&mutex_interrupt_, 0, 1);
  sem_init(&mutex_state_, 0, 1);
  sem_init(&mutex_ping_, 0, 1);
  sem_init(&mutex_ir_, 0, 1);
  
  sem_wait(&mutex_state_);
  sem_wait(&mutex_interrupt_);

  current_power_ = 16;
  left_ = 0;
  right_ = 0;
  angular_ = 0;
  rotate_ = false;
  process_ = false;
  last_cmd_time_ = ros::Time::now();
  interrupt_ = false;

  sem_post(&mutex_interrupt_);
  sem_post(&mutex_state_);

  velocity_sub_ = node_handle_.subscribe("/eddie/command_velocity", 1, &EddieController::velocityCallback, this);
  ping_distances_sub_ = node_handle_.subscribe("/eddie/ping_distances", 1, &EddieController::distanceCallback, this);
  ir_distances_sub_ = node_handle_.subscribe("/eddie/ir_voltages", 1, &EddieController::irCallback, this);
  
  eddie_status_srv_ = node_handle_.advertiseService("emergency_status", &EddieController::getStatus, this);
  
  eddie_drive_power_ = node_handle_.serviceClient<eddiebot_msgs::DriveWithPower > ("drive_with_power");
  eddie_drive_speed_ = node_handle_.serviceClient<eddiebot_msgs::DriveWithSpeed > ("drive_with_speed");
  eddie_acceleration_rate_ = node_handle_.serviceClient<eddiebot_msgs::Accelerate > ("acceleration_rate");
  eddie_turn_ = node_handle_.serviceClient<eddiebot_msgs::Rotate > ("rotate");
  eddie_stop_ = node_handle_.serviceClient<eddiebot_msgs::StopAtDistance > ("stop_at_distance");
  eddie_heading_ = node_handle_.serviceClient<eddiebot_msgs::GetHeading > ("get_heading");
  eddie_reset_ = node_handle_.serviceClient<eddiebot_msgs::ResetEncoder > ("reset_encoder");
  
  setAccelerationRate(acceleration_speed_);
}

void EddieController::velocityCallback(const eddiebot_msgs::Velocity::ConstPtr& message)
{
  float linear = message->linear;
  int16_t angular = message->angular;

  if (linear == 0 && angular == 0)
  {
    stop();
  }
  else if (linear != 0 && angular == 0)
  {
    moveLinear(linear);
  }
  else if (linear == 0 && angular != 0)
  {
    moveAngular(angular);
  }
  else //if (linear!=0 && angular !=0)
  {
    moveLinearAngular(linear, angular);
  }
}

void EddieController::distanceCallback(const eddiebot_msgs::Distances::ConstPtr& message)
{
  sem_wait(&mutex_ping_);
  bool okay = true;
  for (uint i = 0; i < message->value.size(); i++)
  {
    if (message->value[i] != 0 && message->value[i] < 150)
      okay = false;
  }
  ping_distances_okay_ = okay;
  if (!okay)
    stop();
  sem_post(&mutex_ping_);
}

void EddieController::irCallback(const eddiebot_msgs::Voltages::ConstPtr& message)
{
  sem_wait(&mutex_ir_);
  bool okay = true;
  for (uint i = 0; i < message->value.size(); i++)
  {
    if (message->value[i] != 0 && message->value[i] > 1.7)
      okay = false;
  }
  ir_distances_okay_ = okay;
  if (!okay)
    stop();
  sem_post(&mutex_ir_);
}

bool EddieController::getStatus(eddiebot_msgs::GetStatus::Request& req,
  eddiebot_msgs::GetStatus::Response& res)
{
  sem_wait(&mutex_ping_);
  sem_wait(&mutex_ir_);

  if (ping_distances_okay_ && ir_distances_okay_)
    res.okay = true;
  else
    res.okay = false;

  sem_post(&mutex_ping_);
  sem_post(&mutex_ir_);
  return true;
}

void EddieController::stop()
{
  eddiebot_msgs::StopAtDistance dist;
  dist.request.distance = 4;

  sem_wait(&mutex_interrupt_);
  interrupt_ = true;
  sem_post(&mutex_interrupt_);

  for (int i = 0; !eddie_stop_.call(dist) && i < 5; i++)
    ROS_ERROR("ERROR: at trying to stop Eddie. Trying to auto send command again...");

  current_power_ = 0;
}

void EddieController::setAccelerationRate(int rate)
{
  eddiebot_msgs::Accelerate acc;
  acc.request.rate = acceleration_speed_;
  if (!eddie_acceleration_rate_.call(acc))
    ROS_ERROR("ERROR: Failed to set acceleration rate to %d", rate);
}

void EddieController::moveLinear(float linear)
{
  int8_t left, right;
  left = clipPower(left_power_, linear);
  right = clipPower(right_power_, linear);

  sem_wait(&mutex_interrupt_);
  left_ = left;
  right_ = right;
  rotate_ = false;
  process_ = true;
  interrupt_ = true;
  sem_post(&mutex_interrupt_);
}

void EddieController::moveAngular(int16_t angular)
{
  sem_wait(&mutex_interrupt_);
  left_ = 0;
  right_ = 0;
  if (!rotate_)
    current_power_ = 0;
  angular_ = angular;
  rotate_ = true;
  process_ = true;
  interrupt_ = true;
  sem_post(&mutex_interrupt_);
}

void EddieController::moveLinearAngular(float linear, int16_t angular)
{
  int8_t left, right;
  if (angular > 0)
  {
    angular = angular % 360;
    left = clipPower(left_power_, linear);
    right = left - (int8_t) (left * (float) angular / 180);
  }
  else
  {
    angular = angular % 360;
    right = clipPower(right_power_, linear);
    left = right - (int8_t) (right * (float) abs(angular) / 180);
  }
  
  sem_wait(&mutex_interrupt_);
  left_ = left;
  right_ = right;
  rotate_ = false;
  process_ = true;
  interrupt_ = true;
  sem_post(&mutex_interrupt_);
}

void EddieController::drive(int8_t left, int8_t right)
{
  sem_wait(&mutex_execute_);

  sem_wait(&mutex_interrupt_);
  interrupt_ = false;
  bool cancel = interrupt_;
  sem_post(&mutex_interrupt_);

  eddiebot_msgs::DriveWithPower power;
  ros::Time now;
  bool shift = true;
  int8_t previous_power = 0;

  while (ros::ok() && shift && !cancel)
  {
    now = ros::Time::now();
    if ((now.toSec() - last_cmd_time_.toSec()) >= 0.1)
    {
      previous_power = current_power_;
      updatePower(left, right);

      if (abs(left) < abs(right))
      {
        power.request.left = current_power_;
        power.request.right = (int8_t) (current_power_ * ((double) right / left));
      }
      else
      {
        power.request.right = current_power_;
        power.request.left = (int8_t) (current_power_ * ((double) left / right));
      }
      
      if (eddie_drive_power_.call(power))
        last_cmd_time_ = ros::Time::now();
      else
        current_power_ = previous_power;

      if (left != current_power_ && right != current_power_)
        shift = true;
      else
        shift = false;
    }
    ros::spinOnce();
    usleep(1000);
    sem_wait(&mutex_interrupt_);
    cancel = interrupt_;
    sem_post(&mutex_interrupt_);
  }

  sem_post(&mutex_execute_);
}

void EddieController::rotate(int16_t angular)
{
  sem_wait(&mutex_execute_);

  sem_wait(&mutex_interrupt_);
  interrupt_ = false;
  angular_ = 0;
  bool cancel = interrupt_;
  sem_post(&mutex_interrupt_);

  //angular = 0.75 * angular;
  eddiebot_msgs::DriveWithPower power;
  eddiebot_msgs::GetHeading heading;
  ros::Time now;
  bool shift = true, headed = false;
  int16_t init_angle = 0, target_angle;
  int8_t left, right, previous_power;

  eddiebot_msgs::ResetEncoder reset;
  // eddie_reset_.call(reset);
  for (int i = 0; !(headed = eddie_heading_.call(heading)) && i < 5; i++)
    usleep(100);
  if (!headed)
  {
    ROS_ERROR("Unable to get current Heading value. Encoder will now be reseted.");
    sem_post(&mutex_execute_);
    return;
  }
  else
  {
    init_angle = heading.response.heading;
    if (init_angle > 3736) init_angle -= 4096;
  }
  target_angle = init_angle + angular;

  left = angular > 0 ? rotation_power_ : -1 * rotation_power_;
  right = angular > 0 ? -1 * rotation_power_ : rotation_power_;

  while (ros::ok() && shift && !cancel)
  {
    now = ros::Time::now();
    if ((now.toSec() - last_cmd_time_.toSec()) >= 0.1)
    {
      eddie_heading_.call(heading);
      current_angle_ = heading.response.heading;
      if (current_angle_ > 3736)
        current_angle_ -= 4096;

      if (angular > 0 && current_angle_ < init_angle)
        current_angle_ = init_angle > 0 ? current_angle_ + 360 : current_angle_;
      else if (angular < 0 && current_angle_ > init_angle)
        current_angle_ = init_angle < 0 ? current_angle_ - 360 : current_angle_;

      init_angle = current_angle_;

      if (angular > 0 && current_angle_ < target_angle)
        shift = true;
      else if (angular < 0 && current_angle_ > target_angle)
        shift = true;
      else
        shift = false;

      if (!shift)
      {
        eddiebot_msgs::StopAtDistance dist;
        dist.request.distance = 2;
        for (int i = 0; !eddie_stop_.call(dist) && i < 5; i++)
          ROS_ERROR("ERROR: at trying to stop Eddie. Trying to auto send command again...");
        current_power_ = 0;
      }
      else
      {
        previous_power = current_power_;
        updatePower(left, right);
        power.request.left = angular > 0 ? current_power_ : -1 * current_power_;
        power.request.right = angular > 0 ? -1 * current_power_ : current_power_;
        if (eddie_drive_power_.call(power))
          last_cmd_time_ = ros::Time::now();
        else
          current_power_ = previous_power;
      }
    }
    ros::spinOnce();
    usleep(1000);
    sem_wait(&mutex_interrupt_);
    cancel = interrupt_;
    if (!rotate_)
      current_power_ = 0;
    sem_post(&mutex_interrupt_);
  }
  
  sem_post(&mutex_execute_);
}

void EddieController::updatePower(int8_t left, int8_t right)
{
  if (left > 0 && right > 0)
  {
    if (current_power_>-1 * min_power_ && current_power_ < min_power_)
      current_power_ = min_power_;
    else if (current_power_ > acceleration_power_ + left && current_power_ > acceleration_power_ + right)
      current_power_ = left > right ? left - acceleration_power_ : right - acceleration_power_;
    else if (current_power_<-1 * min_power_)
      current_power_ += deceleration_power_ / 10;
    else
      current_power_ += acceleration_power_ / 10;

    if (current_power_ > left || current_power_ > right)
      current_power_ = left > right ? left : right;
  }
  else if (left < 0 && right < 0)
  {
    if (current_power_>-1 * min_power_ && current_power_ < min_power_)
      current_power_ = -1 * min_power_;
    else if (current_power_ < left - acceleration_power_ && current_power_ < right - acceleration_power_)
      current_power_ = left > right ? left - acceleration_power_ : right - acceleration_power_;
    else if (current_power_ > min_power_)
      current_power_ -= deceleration_power_ / 10;
    else
      current_power_ -= acceleration_power_ / 10;

    if (current_power_ < left || current_power_ < right)
      current_power_ = left < right ? left : right;
  }
  else
  {
    if (current_power_ < min_power_)
      current_power_ = min_power_;
    else if (current_power_ < left || current_power_ < right)
      current_power_ += acceleration_power_ / 10;

    if (current_power_ > left && current_power_ > right)
      current_power_ = left > right ? left : right;
  }
}

int8_t EddieController::clipPower(int power_unit, float linear)
{
  int8_t power;
  if (power_unit * abs(linear) > 127)
  {
    if (linear > 0)
      power = 127;
    else
      power = -127;
  }
  else
    power = power_unit * linear;

  return power;
}

int16_t EddieController::clipSpeed(int speed_unit, float linear)
{
  int16_t speed;
  if (speed_unit * abs(linear) > 32767)
  {
    if (linear > 0)
      speed = 32767;
    else
      speed = -32767;
  }
  else
    speed = speed_unit * linear;

  return speed;
}

void EddieController::execute()
{
  ros::Rate rate(1000);
  while (ros::ok())
  {
    sem_wait(&mutex_interrupt_);
    bool ex = process_;
    sem_post(&mutex_interrupt_);
    if (ex)
    {
      sem_wait(&mutex_interrupt_);
      process_ = false;
      int8_t l = left_;
      int8_t r = right_;
      bool rot = rotate_;
      int16_t angular = angular_;
      sem_post(&mutex_interrupt_);
      if (rot)
        rotate(angular);
      else
        drive(l, r);
    }

    ros::spinOnce();
    rate.sleep();
  }
}

/*
 * 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "eddie_controller");
  EddieController controller;
  controller.execute();

  return (EXIT_SUCCESS);
}

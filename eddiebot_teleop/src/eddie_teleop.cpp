/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Haikal Pribadi <haikal.pribadi@gmail.com>
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

#include "eddiebot_teleop/eddie_teleop.h"

EddieTeleop::EddieTeleop(std::shared_ptr<rclcpp::Node> node_handle) :
  node_handle_(node_handle), linear_(0), angular_(0), l_scale_(2.0), a_scale_(2.0)
{
  velocity_pub_ = node_handle_->create_publisher<eddiebot_msgs::msg::Velocity>(
      "/eddie/command_velocity", 1);
  keystroke_pub_ = node_handle_->create_publisher<eddiebot_msgs::msg::KeyStroke>(
      "/eddie/key_stroke", 1);

  node_handle_->get_parameter_or("angular_scale", a_scale_, a_scale_);
  node_handle_->get_parameter_or("linear_scale", l_scale_, l_scale_);
}

void EddieTeleop::keyLoop()
{
  char c, cb;
  bool move = false;

  //get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof (struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  int flags;
  flags = fcntl(kfd, F_GETFL);
  flags |= O_NONBLOCK;
  fcntl(0, F_SETFL, flags);

  RCLCPP_INFO(node_handle_->get_logger(), "Reading from keyboard");
  RCLCPP_INFO(node_handle_->get_logger(), "================================================");
  RCLCPP_INFO(node_handle_->get_logger(), "Use arrow keys to navigate and space bar to stop");

  while (true)
  {
    cb = -1;
    usleep(100000); //in microseconds
    //get the next event from the keyboard
    //by getting the very last value stored in the keyboard buffer
    while (read(kfd, &c, 1) >= 0)
    {
      cb = c;
    }
    c = cb;

    linear_ = angular_ = 0;
    //RCLCPP_INFO(node_handle_->get_logger(), "value: 0x%02X\n", c);
    //RCLCPP_INFO(node_handle_->get_logger(), "KEY PRESSED: %s", &c);
    switch (c)
    {
      case KEYCODE_L:
        RCLCPP_DEBUG(node_handle_->get_logger(), "LEFT");
        angular_ = -10;
        //linear_ = 1.0; //to test movement with linear and angular
        move = true;
        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(node_handle_->get_logger(), "RIGHT");
        angular_ = 10;
        //linear_ = 1.0; //to test movement with linear and angular
        move = true;
        break;
      case KEYCODE_U:
        RCLCPP_DEBUG(node_handle_->get_logger(), "UP");
        linear_ = 1.0;
        move = true;
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(node_handle_->get_logger(), "DOWN");
        linear_ = -1.0;
        move = true;
        break;
      case ' ':
        RCLCPP_DEBUG(node_handle_->get_logger(), "STOP");
        linear_ = 0;
        move = true;
        break;
    }

    if (move)
    {
      eddiebot_msgs::msg::Velocity vel;
      vel.angular = angular_ * a_scale_;
      vel.linear = linear_ * l_scale_;
      velocity_pub_->publish(vel);
      move = false;
    }
    if(c!=-1){
      eddiebot_msgs::msg::KeyStroke key;
      key.keycode = c;
      keystroke_pub_->publish(key);
    }
  }
}

void quit(int sig)
{
  (void) sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node_handle = rclcpp::Node::make_shared("eddie_teleop");
  EddieTeleop eddie_teleop(node_handle);
  signal(SIGINT, quit);
  eddie_teleop.keyLoop();

  return (EXIT_SUCCESS);
}

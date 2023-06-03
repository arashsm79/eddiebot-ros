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

bool running = true;

EddieTeleop::EddieTeleop(std::shared_ptr<rclcpp::Node> node_handle)
    : node_handle_(node_handle), linear_(0), angular_(0), l_scale_(2.0),
      a_scale_(2.0) {
  velocity_pub_ = node_handle_->create_publisher<eddiebot_msgs::msg::Velocity>(
      "/eddie/simple_velocity", 1);
  keystroke_pub_ =
      node_handle_->create_publisher<eddiebot_msgs::msg::KeyStroke>(
          "/eddie/key_stroke", 1);

  node_handle_->get_parameter_or("angular_scale", a_scale_, a_scale_);
  node_handle_->get_parameter_or("linear_scale", l_scale_, l_scale_);
}

void EddieTeleop::spin() { rclcpp::spin(node_handle_); }

int EddieTeleop::keyLoop() {
  char c = 0;

  std::thread{std::bind(&EddieTeleop::spin, this)}.detach();

  RCLCPP_INFO(node_handle_->get_logger(), "Reading from keyboard");
  RCLCPP_INFO(node_handle_->get_logger(),
              "================================================");
  RCLCPP_INFO(node_handle_->get_logger(),
              "Use arrow keys to navigate and space bar to stop");

  while (running) {
    // get the next event from the keyboard
    try {
      c = input_.readOne();
    } catch (const std::runtime_error &) {
      perror("read():");
      return -1;
    }

    linear_ = angular_ = 0;
    bool valid_key = true;
    // RCLCPP_INFO(node_handle_->get_logger(), "value: 0x%02X\n", c);
    // RCLCPP_INFO(node_handle_->get_logger(), "KEY PRESSED: %s", &c);
    switch (c) {
    case KEYCODE_L:
      RCLCPP_DEBUG(node_handle_->get_logger(), "LEFT");
      angular_ = -10;
      // linear_ = 1.0; //to test movement with linear and angular
      break;
    case KEYCODE_R:
      RCLCPP_DEBUG(node_handle_->get_logger(), "RIGHT");
      angular_ = 10;
      // linear_ = 1.0; //to test movement with linear and angular
      break;
    case KEYCODE_U:
      RCLCPP_DEBUG(node_handle_->get_logger(), "UP");
      linear_ = 1.0;
      break;
    case KEYCODE_D:
      RCLCPP_DEBUG(node_handle_->get_logger(), "DOWN");
      linear_ = -1.0;
      break;
    case ' ':
      RCLCPP_DEBUG(node_handle_->get_logger(), "STOP");
      linear_ = 0;
      break;
    default:
      break;
      valid_key = false;
    }

    if(valid_key) {
      eddiebot_msgs::msg::Velocity vel;
      vel.angular = angular_ * a_scale_;
      vel.linear = linear_ * l_scale_;
      velocity_pub_->publish(vel);
      eddiebot_msgs::msg::KeyStroke key;
      key.keycode = c;
      keystroke_pub_->publish(key);
    }
  }
  return 0;
}

void quit(int sig) {
  (void)sig;
  running = false;
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node_handle = rclcpp::Node::make_shared("eddie_teleop");
  EddieTeleop eddie_teleop(node_handle);
  signal(SIGINT, quit);
  int rc = eddie_teleop.keyLoop();

  return (rc);
}

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

#ifndef _EDDIE_TELEOP_H
#define _EDDIE_TELEOP_H

#include <fcntl.h>
#include <functional>
#include <signal.h>
#include <stdexcept>
#include <stdio.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "eddiebot_msgs/msg/key_stroke.hpp"
#include "eddiebot_msgs/msg/velocity.hpp"
#include "rclcpp/rclcpp.hpp"

#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44

int kfd = 0;
struct termios cooked, raw;

class KeyboardReader final {
public:
  KeyboardReader() {
    // get the console in raw mode
    if (tcgetattr(0, &cooked_) < 0) {
      throw std::runtime_error("Failed to get old console mode");
    }
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    raw.c_cc[VTIME] = 1;
    raw.c_cc[VMIN] = 0;
    if (tcsetattr(0, TCSANOW, &raw) < 0) {
      throw std::runtime_error("Failed to set new console mode");
    }
  }

  char readOne() {
    char c = 0;

    int rc = read(0, &c, 1);
    if (rc < 0) {
      throw std::runtime_error("read failed");
    }
    return c;
  }

  ~KeyboardReader() { tcsetattr(0, TCSANOW, &cooked_); }

private:
  struct termios cooked_;
};

class EddieTeleop {
public:
  EddieTeleop(std::shared_ptr<rclcpp::Node>);
  int keyLoop();

private:
  std::shared_ptr<rclcpp::Node> node_handle_;
  rclcpp::Publisher<eddiebot_msgs::msg::Velocity>::SharedPtr velocity_pub_;
  rclcpp::Publisher<eddiebot_msgs::msg::KeyStroke>::SharedPtr keystroke_pub_;
  KeyboardReader input_;
  void spin();
  float linear_, angular_;
  double l_scale_, a_scale_;
};

#endif /* _EDDIE_TELEOP_H */

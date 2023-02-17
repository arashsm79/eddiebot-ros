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

#include "eddiebot_bringup/eddie.h"
#include <iostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>

typedef std::map<std::string, unsigned char[6]> CommandMap;

// maximum number of tries to contact serial port
#define MAX_N_TRIES 3

Eddie::Eddie(std::shared_ptr<rclcpp::Node> node_handle)
    : GPIO_COUNT(10), ADC_PIN_COUNT(8), DIGITAL_PIN_COUNT(10),
      AUXILIARY_POWER_RELAY_COUNT(3), PARALLAX_MAX_BUFFER(256),
      ENCODER_COUNT(2), ADC_VOLTAGE_MULTIPLIER(1 / 819),
      BATTERY_VOLTAGE_DIVIDER(3.21), MOTOR_POWER_STOP(0),
      MOTOR_POWER_MAX_FORWARD(127), MOTOR_POWER_MAX_REVERSE(-127),
      TRAVEL_SPEED_MAX_FORWARD(32767), TRAVEL_SPEED_MAX_REVERSE(-32767),
      TRAVEL_MAX_SPEED(65535), RELAY_33V_PIN_NUMBER(17),
      RELAY_5V_PIN_NUMBER(17), RELAY_12V_PIN_NUMBER(18),
      PACKET_TERMINATOR('\r'), PARAMETER_DELIMITER(' '), ERROR("ERROR"),
      DEFAULT_WHEEL_RADIUS(0.0762), DEFAULT_TICKS_PER_REVOLUTION(36),
      GET_VERSION_STRING("VER"), SET_GPIO_DIRECTION_OUT_STRING("OUT"),
      SET_GPIO_DIRECTION_IN_STRING("IN"), SET_GPIO_STATE_HIGH_STRING("HIGH"),
      SET_GPIO_STATE_LOW_STRING("LOW"), GET_GPIO_STATE_STRING("READ"),
      GET_ADC_VALUE_STRING("ADC"), GET_PING_VALUE_STRING("PING"),
      SET_DRIVE_POWER_STRING("GO"), SET_DRIVE_SPEED_STRING("GOSPD"),
      SET_DRIVE_DISTANCE_STRING("TRVL"), SET_STOP_DISTANCE_STRING("STOP"),
      SET_ROTATE_STRING("TURN"), GET_CURRENT_SPEED_STRING("SPD"),
      GET_CURRENT_HEADING_STRING("HEAD"), GET_ENCODER_TICKS_STRING("DIST"),
      RESET_ENCODER_TICKS_STRING("RST"), SET_RAMPING_VALUE_STRING("ACC"),
      FLUSH_BUFFERS_STRING("\r\r\r"), node_handle_(node_handle) {
  sem_init(&mutex, 0, 1);
  ping_pub_ = node_handle_->create_publisher<eddiebot_msgs::msg::Ping>(
      "/eddie/ping_data", 1);
  adc_pub_ = node_handle_->create_publisher<eddiebot_msgs::msg::ADC>(
      "/eddie/adc_data", 1);
  encoder_pub_ = node_handle_->create_publisher<eddiebot_msgs::msg::Encoders>(
      "/eddie/encoders_data", 1);

  accelerate_srv_ =
      node_handle_->create_service<eddiebot_msgs::srv::Accelerate>(
          "acceleration_rate",
          std::bind(&Eddie::accelerate, this, std::placeholders::_1,
                    std::placeholders::_2));
  drive_with_distance_srv_ =
      node_handle_->create_service<eddiebot_msgs::srv::DriveWithDistance>(
          "drive_with_distance",
          std::bind(&Eddie::driveWithDistance, this, std::placeholders::_1,
                    std::placeholders::_2));
  drive_with_power_srv_ =
      node_handle_->create_service<eddiebot_msgs::srv::DriveWithPower>(
          "drive_with_power",
          std::bind(&Eddie::driveWithPower, this, std::placeholders::_1,
                    std::placeholders::_2));
  drive_with_speed_srv_ =
      node_handle_->create_service<eddiebot_msgs::srv::DriveWithSpeed>(
          "drive_with_speed",
          std::bind(&Eddie::driveWithSpeed, this, std::placeholders::_1,
                    std::placeholders::_2));
  get_distance_srv_ =
      node_handle_->create_service<eddiebot_msgs::srv::GetDistance>(
          "get_distance",
          std::bind(&Eddie::getDistance, this, std::placeholders::_1,
                    std::placeholders::_2));
  get_heading_srv_ =
      node_handle_->create_service<eddiebot_msgs::srv::GetHeading>(
          "get_heading",
          std::bind(&Eddie::getHeading, this, std::placeholders::_1,
                    std::placeholders::_2));
  get_speed_srv_ = node_handle_->create_service<eddiebot_msgs::srv::GetSpeed>(
      "get_speed", std::bind(&Eddie::getSpeed, this, std::placeholders::_1,
                             std::placeholders::_2));
  reset_encoder_srv_ =
      node_handle_->create_service<eddiebot_msgs::srv::ResetEncoder>(
          "reset_encoder",
          std::bind(&Eddie::resetEncoder, this, std::placeholders::_1,
                    std::placeholders::_2));
  rotate_srv_ = node_handle_->create_service<eddiebot_msgs::srv::Rotate>(
      "rotate", std::bind(&Eddie::rotate, this, std::placeholders::_1,
                          std::placeholders::_2));
  stop_at_distance_srv_ =
      node_handle_->create_service<eddiebot_msgs::srv::StopAtDistance>(
          "stop_at_distance",
          std::bind(&Eddie::stopAtDistance, this, std::placeholders::_1,
                    std::placeholders::_2));

  std::string port =
      node_handle_
          ->get_parameter_or("serial_port",
                             rclcpp::Parameter("serial_port", "/dev/ttyUSB0"))
          .as_string();
  initialize(port);

  command(RESET_ENCODER_TICKS_STRING);
}

Eddie::~Eddie() {
  command("STOP 0");
  close(tty_fd);
}

void Eddie::initialize(std::string port) {
  RCLCPP_INFO(node_handle_->get_logger(),
              "Initializing Parallax board serial port connection");

  memset(&tio, 0, sizeof(tio));
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_cflag = CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
  tio.c_lflag = 0;
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 5;

  tty_fd = open(port.data(), O_RDWR | O_NONBLOCK);
  cfsetospeed(&tio, B115200); // 115200 baud
  cfsetispeed(&tio, B115200); // 115200 baud

  tcsetattr(tty_fd, TCSANOW, &tio);
  usleep(100000);
}

std::string Eddie::command(std::string str) {
  sem_wait(&mutex);
  ssize_t _written;
  (void) _written;
  std::stringstream result("");
  int count = 0;
  unsigned char c;
  int size = str.size() + 1;
  unsigned char command[size];

  for (int i = 0; i < size - 1; i++) {
    command[i] = str[i];
  }
  command[size - 1] = PACKET_TERMINATOR; // Having exces terminator is okay,
                                         // it's good to guarantee

  _written = write(tty_fd, command, size);
  while (read(tty_fd, &c, 1) <= 0) {
    usleep(1000);
    count++;
    if (count >= 80) {
      RCLCPP_ERROR(node_handle_->get_logger(),
                   "ERROR: NO PARALLAX EDDIE ROBOT IS CONNECTED.");
      RCLCPP_ERROR(node_handle_->get_logger(), "%s", str.c_str());
      break;
    }
  }
  if (count < 80) {
    result << c;
    count = 0;
    while (c != '\r') {
      if (read(tty_fd, &c, 1) > 0)
        result << c;
    }
  }
  sem_post(&mutex);
  return result.str();
}

std::string Eddie::generateCommand(std::string str1) {
  std::stringstream ss;
  ss << str1 << PACKET_TERMINATOR;
  return ss.str();
}

std::string Eddie::generateCommand(std::string str1, int num1) {
  std::stringstream ss;
  ss << str1 << PARAMETER_DELIMITER << intToHexString(num1)
     << PACKET_TERMINATOR;
  return ss.str();
}

std::string Eddie::generateCommand(std::string str1, int num1, int num2) {
  std::stringstream ss;
  ss << str1 << PARAMETER_DELIMITER << intToHexString(num1)
     << PARAMETER_DELIMITER << intToHexString(num2) << PACKET_TERMINATOR;
  return ss.str();
}

std::string Eddie::intToHexString(int num) {
  std::stringstream ss;
  ss << std::hex << num;
  return ss.str();
}

eddiebot_msgs::msg::Ping Eddie::getPingData() {
  std::string result = command(GET_PING_VALUE_STRING);
  // e.g. result = "133 3C9 564 0F9 29B 0F0 31A 566 1E0 A97\r";
  eddiebot_msgs::msg::Ping ping_data;
  if (result.size() <= 1) {
    ping_data.status = "EMPTY";
    return ping_data;
  } else if (result.size() >= 6) {
    if (result.substr(0, 5) == "ERROR") {
      ping_data.status = result;
      return ping_data;
    }
  }
  ping_data.status = "SUCCESS";
  std::stringstream value;
  uint16_t data;
  for (ushort i = 0; i < result.size(); i += 4) {
    if (result[i] == '\r')
      break;
    value << std::hex << result.substr(i, 3);
    value >> data;
    ping_data.value.push_back(data);
    value.str(std::string());
    value.clear();
  }
  return ping_data;
}

eddiebot_msgs::msg::ADC Eddie::getADCData() {
  std::string result = command(GET_ADC_VALUE_STRING);
  // e.g.  result = "9C7 11E E4E 5AB 20F 97B 767 058\r";
  eddiebot_msgs::msg::ADC adc_data;
  if (result.size() <= 1) {
    adc_data.status = "EMPTY";
    return adc_data;
  } else if (result.size() >= 6) {
    if (result.substr(0, 5) == "ERROR") {
      adc_data.status = result;
      return adc_data;
    }
  }
  adc_data.status = "SUCCESS";
  std::stringstream value;
  uint16_t data;
  for (ushort i = 0; i < result.size(); i += 4) {
    if (result[i] == '\r')
      break;
    value << std::hex << result.substr(i, 3);
    value >> data;
    adc_data.value.push_back(data);
    value.str(std::string());
    value.clear();
  }
  return adc_data;
}

bool Eddie::getEncodersData(eddiebot_msgs::msg::Encoders &data) {
  std::string cmd = GET_ENCODER_TICKS_STRING;
  std::string cmd_response = command(cmd);

  if (cmd_response.substr(0, 5) != "ERROR" && cmd_response.size() >= 18) {
    std::stringstream value;
    unsigned int tmp;
    value << std::hex << cmd_response.substr(0, 8);
    value >> tmp;
    data.left = static_cast<int>(tmp);

    value.str(std::string());
    value.clear();
    value << std::hex << cmd_response.substr(9, 8);
    value >> tmp;
    data.right = static_cast<int>(tmp);

    return true;
  } else
    return false;
}

void Eddie::publishPingData() { ping_pub_->publish(getPingData()); }

void Eddie::publishADCData() { adc_pub_->publish(getADCData()); }

void Eddie::publishEncodersData() {
  int cnt = 0;
  eddiebot_msgs::msg::Encoders encoders_data;

  if (!getEncodersData(encoders_data) && cnt < MAX_N_TRIES)
    cnt++;

  if (cnt == MAX_N_TRIES) {
    RCLCPP_ERROR(node_handle_->get_logger(),
                 "[Eddie::publishEncodersData] Cannot contact serial port");
    exit(-1);
  } else
    encoder_pub_->publish(encoders_data);
}

bool Eddie::accelerate(
    const std::shared_ptr<eddiebot_msgs::srv::Accelerate::Request> req,
    std::shared_ptr<eddiebot_msgs::srv::Accelerate::Response> res) {
  // this feature does not need to validate the parameters due the limited range
  // of parameter data type
  std::string cmd;
  (void) res;
  cmd = generateCommand(SET_RAMPING_VALUE_STRING, req->rate);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}

bool Eddie::driveWithDistance(
    const std::shared_ptr<eddiebot_msgs::srv::DriveWithDistance::Request> req,
    std::shared_ptr<eddiebot_msgs::srv::DriveWithDistance::Response> res) {
  // this feature does not need to validate the parameters due the limited range
  // of parameter data type
  (void) res;
  std::string cmd;
  cmd = generateCommand(SET_DRIVE_DISTANCE_STRING, req->distance, req->speed);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}

bool Eddie::driveWithPower(
    const std::shared_ptr<eddiebot_msgs::srv::DriveWithPower::Request> req,
    std::shared_ptr<eddiebot_msgs::srv::DriveWithPower::Response> res) {
  (void) res;
  if (req->left > MOTOR_POWER_MAX_FORWARD ||
      req->right > MOTOR_POWER_MAX_FORWARD ||
      req->left < MOTOR_POWER_MAX_REVERSE ||
      req->right < MOTOR_POWER_MAX_REVERSE) {
    return false;
  }
  std::string cmd;
  cmd = generateCommand(SET_DRIVE_POWER_STRING, req->left, req->right);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else {
    RCLCPP_ERROR(node_handle_->get_logger(), "%s", cmd_response.data());
    return false;
  }
}

bool Eddie::driveWithSpeed(
    const std::shared_ptr<eddiebot_msgs::srv::DriveWithSpeed::Request> req,
    std::shared_ptr<eddiebot_msgs::srv::DriveWithSpeed::Response> res) {
  (void) res;
  if (req->left > TRAVEL_SPEED_MAX_FORWARD ||
      req->right > TRAVEL_SPEED_MAX_FORWARD ||
      req->left < TRAVEL_SPEED_MAX_REVERSE ||
      req->right < TRAVEL_SPEED_MAX_REVERSE) {
    return false;
  }
  std::string cmd;
  cmd = generateCommand(SET_DRIVE_SPEED_STRING, req->left, req->right);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}

bool Eddie::getDistance(
    const std::shared_ptr<eddiebot_msgs::srv::GetDistance::Request> req,
    std::shared_ptr<eddiebot_msgs::srv::GetDistance::Response> res) {
  (void) res;
  (void) req;
  std::string cmd = GET_ENCODER_TICKS_STRING;
  std::string cmd_response = command(cmd);
  if (cmd_response.substr(0, 5) != "ERROR" && cmd_response.size() >= 18) {
    std::stringstream value;
    value << std::hex << cmd_response.substr(0, 8);
    value >> res->left;
    value.str(std::string());
    value.clear();
    value << std::hex << cmd_response.substr(9, 8);
    value >> res->right;
    return true;
  } else
    return false;
}

bool Eddie::getHeading(
    const std::shared_ptr<eddiebot_msgs::srv::GetHeading::Request> req,
    std::shared_ptr<eddiebot_msgs::srv::GetHeading::Response> res) {
  (void) res;
  (void) req;
  std::string cmd = GET_CURRENT_HEADING_STRING;
  std::string cmd_response = command(cmd);
  if (cmd_response.substr(0, 5) != "ERROR" && cmd_response.size() >= 4) {
    std::stringstream value;
    value << std::hex << cmd_response.substr(0, 3);
    value >> res->heading;
    return true;
  } else
    return false;
}

bool Eddie::getSpeed(
    const std::shared_ptr<eddiebot_msgs::srv::GetSpeed::Request> req,
    std::shared_ptr<eddiebot_msgs::srv::GetSpeed::Response> res) {
  (void) res;
  (void) req;
  std::string cmd = GET_CURRENT_SPEED_STRING;
  std::string cmd_response = command(cmd);
  if (cmd_response.substr(0, 5) != "ERROR" && cmd_response.size() >= 10) {
    uint16_t tmp;
    std::stringstream value;
    value << std::hex << cmd_response.substr(0, 4);
    value >> tmp;
    res->left = static_cast<uint16_t>(tmp);

    value.str(std::string());
    value.clear();
    value << std::hex << cmd_response.substr(5, 4);
    value >> tmp;
    res->right = static_cast<uint16_t>(tmp);

    return true;
  } else
    return false;
}

bool Eddie::resetEncoder(
    const std::shared_ptr<eddiebot_msgs::srv::ResetEncoder::Request> req,
    std::shared_ptr<eddiebot_msgs::srv::ResetEncoder::Response> res) {
  (void) res;
  (void) req;
  std::string cmd = RESET_ENCODER_TICKS_STRING;
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}

bool Eddie::rotate(
    const std::shared_ptr<eddiebot_msgs::srv::Rotate::Request> req,
    std::shared_ptr<eddiebot_msgs::srv::Rotate::Response> res) {
  (void) res;
  std::string cmd;
  cmd = generateCommand(SET_ROTATE_STRING, req->angle, req->speed);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}

bool Eddie::stopAtDistance(
    const std::shared_ptr<eddiebot_msgs::srv::StopAtDistance::Request> req,
    std::shared_ptr<eddiebot_msgs::srv::StopAtDistance::Response> res) {
  (void) res;
  std::string cmd;
  cmd = generateCommand(SET_STOP_DISTANCE_STRING, req->distance);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node_handle = rclcpp::Node::make_shared("parallax_board");
  RCLCPP_INFO(node_handle->get_logger(), "Parallax Board booting up");
  Eddie eddie(node_handle); // set port to connect to Parallax controller board
  rclcpp::Rate loop_rate(5);

  while (rclcpp::ok()) {
    // eddie.publishPingData();
    // eddie.publishADCData();
    eddie.publishEncodersData();

    rclcpp::spin_some(node_handle);
    loop_rate.sleep();
  }

  return 0;
}

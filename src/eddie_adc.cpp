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

#include "eddiebot_bringup/eddie_adc.h"
#include <rclcpp/executors.hpp>

//=============================================================================//
// This class is provided as a template for future features on the ADC sensors
// // The callback function may be modified to adapt to custom configurations of
// // ADC sensors. Current (default) settings are for a set of IR distance //
// sensors and a battery sensor at the very end //
//=============================================================================//

EddieADC::EddieADC(std::shared_ptr<rclcpp::Node> node_handle)
    : node_handle_(node_handle), ADC_VOLTAGE_DIVIDER(819),
      BATTERY_VOLTAGE_MULTIPLIER(3.21) {
  ir_pub_ = node_handle_->create_publisher<eddiebot_msgs::msg::Voltages>(
      "/eddie/ir_voltages", 1);
  battery_pub_ =
      node_handle_->create_publisher<eddiebot_msgs::msg::BatteryLevel>(
          "/eddie/battery_level", 1);
  adc_sub_ = node_handle_->create_subscription<eddiebot_msgs::msg::ADC>(
      "/eddie/adc_data", 1,
      std::bind(&EddieADC::adcCallback, this, std::placeholders::_1));
}

void EddieADC::adcCallback(
    const eddiebot_msgs::msg::ADC::ConstSharedPtr message) {
  eddiebot_msgs::msg::Voltages voltages;
  eddiebot_msgs::msg::BatteryLevel level;
  double v, l;
  if (message->status.substr(0, 5) ==
      "ERROR") // ERROR messages may be longer than 5 if in VERBOSE mode
  {
    RCLCPP_ERROR(node_handle_->get_logger(), "ERROR: Unable to read ADC data for IR");
    return;
  }

  uint i;
  for (i = 0; i < message->value.size() - 1; i++) {
    v = message->value[i];
    if (v > 10) {
      v = v / ADC_VOLTAGE_DIVIDER;
      voltages.value.push_back(v);
    }
  }
  l = message->value[i];
  l = l / ADC_VOLTAGE_DIVIDER * BATTERY_VOLTAGE_MULTIPLIER;
  level.value = l;
  ir_pub_->publish(voltages);
  battery_pub_->publish(level);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node_handle = rclcpp::Node::make_shared("parallax_adc");
  EddieADC adc(node_handle);
  rclcpp::spin(node_handle);

  return 0;
}

/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Haikal Pribadi <haikal.pribadi@gmail.com>
 * Copyright (c) 2018, Zeyu Zhang <zeyuz@outlook.com>
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

#ifndef _EDDIE_ADC_H
#define	_EDDIE_ADC_H

#include <ros/ros.h>
#include <eddiebot_msgs/ADC.h>
#include <eddiebot_msgs/BatteryLevel.h>
#include <eddiebot_msgs/Voltages.h>

//=============================================================================//
// This class is provided as a template for future features on the ADC sensors //
// The callback function may be modified to adapt to custom configurations of  //
// ADC sensors. Current (default) settings are for a set of IR distance        //
// sensors and a battery sensor at the very end                                //
//=============================================================================//

class EddieADC
{
public:
  EddieADC();

private:
  ros::NodeHandle node_handle_;
  ros::Publisher ir_pub_;
  ros::Publisher battery_pub_;
  ros::Subscriber adc_sub_;
  const double ADC_VOLTAGE_DIVIDER;
  const double BATTERY_VOLTAGE_MULTIPLIER;

  void adcCallback(const eddiebot_msgs::ADC::ConstPtr& message);
};

#endif	/* _EDDIE_ADC_H */



// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER__VESC_DRIVER_HPP_
#define VESC_DRIVER__VESC_DRIVER_HPP_

#include <optional>
#include <memory>
#include <string>
#include <fstream>
#include <json.hpp>

#include <DDSSubscriber.hpp>
#include <DDSPublisher.hpp>
#include <DDSLogger.hpp>

#include <VescStatePubSubTypes.h>
#include <VescStateStampedPubSubTypes.h>
#include <VescImuPubSubTypes.h>
#include <VescImuStampedPubSubTypes.h>
#include <Float64PubSubTypes.h>
#include <ImuPubSubTypes.h>

#include "vesc_driver/vesc_interface.hpp"
#include "vesc_driver/vesc_packet.hpp"

namespace vesc_driver
{


using std_msgs::msg::Float64;
using vesc_msgs::msg::VescState;
using vesc_msgs::msg::VescStateStamped;
using vesc_msgs::msg::VescImuStamped;
using sensor_msgs::msg::Imu;

class VescDriver
{
  // limits on VESC commands
  struct CommandLimit
  {

    CommandLimit(
         const std::string & str,
         const std::optional<double> & min_lower = std::optional<double>(),
         const std::optional<double> & max_upper = std::optional<double>()
       );

    double clip(double value);
    std::string name;
    std::optional<double> lower;
    std::optional<double> upper;
  };


public:
  // driver modes (possible states)
  typedef enum
  {
     MODE_INITIALIZING,
     MODE_OPERATING
  }
  driver_mode_t;

   VescDriver(const std::string& configFile);
   VescDriver(const std::string& configFile, std::unique_ptr<VescInterface>&& vescPtr);
   ~VescDriver();

   std::string getPort() const { return port_; }
   double getSphereRadius() const { return sphere_radius_m; }
   double getTireRadius() const { return tire_radius_m; }
   driver_mode_t getDriverMode() const { return driver_mode_; }
   CommandLimit getDutyCycleLimit() const { return duty_cycle_limit_; }
   CommandLimit getCurrentLimit() const { return current_limit_; }
   CommandLimit getBrakeLimit() const { return brake_limit_; }
   CommandLimit getSpeedLimit() const { return speed_limit_; }
   CommandLimit getPositionLimit() const { return position_limit_; }
   CommandLimit getServoLimit() const { return servo_limit_; }

   //Used for testing
   void setDriverMode(driver_mode_t d) { driver_mode_ = d; }

private:
  void ParseAndInitializeDriver(const std::string& configFile);
  // interface to the VESC
  std::unique_ptr<VescInterface> vesc_;
  void vescPacketCallback(const std::shared_ptr<VescPacket const> & packet);
  void vescErrorCallback(const std::string & error);


  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;


  //TODO enable subscribers as needed in the future
  DDSPublisher<VescStateStamped, vesc_msgs::msg::VescStateStampedPubSubType> state_pub_;
  DDSPublisher<VescImuStamped, vesc_msgs::msg::VescImuStampedPubSubType> imu_pub_;
  DDSPublisher<Imu, sensor_msgs::msg::ImuPubSubType> imu_std_pub_;
  DDSPublisher<Float64, std_msgs::msg::Float64PubSubType> servo_sensor_pub_;

  //DDSSubscriber<Float64, std_msgs::msg::Float64PubSubType> duty_cycle_sub_;
  //DDSSubscriber<Float64, std_msgs::msg::Float64PubSubType> current_sub_;
  //DDSSubscriber<Float64, std_msgs::msg::Float64PubSubType> brake_sub_;
  DDSSubscriber<Float64, std_msgs::msg::Float64PubSubType> speed_sub_;
  //DDSSubscriber<Float64, std_msgs::msg::Float64PubSubType> position_sub_;
  //DDSSubscriber<Float64, std_msgs::msg::Float64PubSubType> servo_sub_;

  double tire_radius_m;
  double sphere_radius_m;

  std::string port_;

  std::thread speedThread_;
  std::thread timerThread_;
  std::atomic_bool timerThreadRunning_;

  DDSLogger logger;

  void runTimer(const std::chrono::milliseconds interval);

  int32_t getCurrentTimeInSeconds();

  void parseConfigFile(const std::string& configFile);

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by vesc
  int fw_version_minor_;                ///< firmware minor version reported by vesc

  //TODO enable as needed in the future
  // DDS callbacks
  //void brakeCallback(const Float64 brake);
  //void currentCallback(const Float64 current);
  //void dutyCycleCallback(const Float64 duty_cycle);
  //void positionCallback(const Float64 position);
  //void servoCallback(const Float64 servo);
  void speedCallback(const Float64 speed);
  void timerCallback();
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER__VESC_DRIVER_HPP_

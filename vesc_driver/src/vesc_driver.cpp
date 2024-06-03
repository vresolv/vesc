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

#include "vesc_driver/vesc_driver.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

namespace vesc_driver
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;
using sensor_msgs::msg::Imu;

VescDriver::VescDriver(const std::string& configFilePath)
  : vesc_(
    std::string(),
    std::bind(&VescDriver::vescPacketCallback, this, _1),
    std::bind(&VescDriver::vescErrorCallback, this, _1)),
  duty_cycle_limit_("duty_cycle", -1.0, 1.0),
  current_limit_("current"),
  brake_limit_("brake"),
  speed_limit_("speed"),
  position_limit_("position"),
  servo_limit_("servo", 0.0, 1.0),
  driver_mode_(MODE_INITIALIZING),
  fw_version_major_(-1),
  fw_version_minor_(-1),
  state_pub_("VescStateStamped", "sensors/core"),
  imu_pub_("VescImuStamped", "sensors/imu"),
  imu_std_pub_("Imu", "sensors/imu/raw"),
  servo_sensor_pub_("Servo_Sensor", "sensors/servo_position_command"),
  //duty_cycle_sub_(std::bind(&VescDriver::dutyCycleCallback, this, _1), "Duty_Cycle", "commands/motor/duty_cycle"),
  //current_sub_(std::bind(&VescDriver::currentCallback, this, _1), "Current", "commands/motor/current"),
  //brake_sub_(std::bind(&VescDriver::brakeCallback, this, _1), "Brake", "commands/motor/brake"),
  speed_sub_(std::bind(&VescDriver::speedCallback, this, _1), "Speed", "commands/motor/speed"),
  //position_sub_(std::bind(&VescDriver::positionCallback, this, _1), "Position", "commands/motor/position"),
  //servo_sub_(std::bind(&VescDriver::servoCallback, this, _1), "Servo", "commands/servo/position"),
  timerThreadRunning_(true)
{
  // get vesc serial port address
  //std::string port = declare_parameter<std::string>("port", "");

   parseConfigFile(configFilePath);

  // attempt to connect to the serial port
  try
  {
    vesc_.connect(port_);
  } catch (SerialException e)
  {
     std::cout << "Failed to connect to the VESC on port " << port_ << ", " << e.what() << std::endl;
     std::exit(EXIT_FAILURE);
  }

  state_pub_.init();
  imu_pub_.init();
  imu_std_pub_.init();
  servo_sensor_pub_.init();

  timerThread_ = std::thread(&VescDriver::runTimer, this, std::chrono::milliseconds(20));

  //TODO: Check if we need other subscribers.
  //Since we only send the speed values to the vesc motor right now
  if (speed_sub_.init())
    speed_sub_.run();
}

VescDriver::~VescDriver()
{
   timerThreadRunning_ = false;
   if (timerThread_.joinable())
      timerThread_.join();
}

void VescDriver::parseConfigFile(const std::string& configFilePath)
{
   std::ifstream configFile(configFilePath);
   if (!configFile.is_open())
   {
      std::cout << "Could not open VESC config file: " + configFilePath << std::endl;
      std::exit(EXIT_FAILURE);
   }

   try
   {
      nlohmann::json jsonConf;
      configFile >> jsonConf;

      if (jsonConf.contains("port"))
      {
         port_ = jsonConf["port"];
      }

      if (jsonConf.contains("brake_max"))
      {
         brake_limit_.upper = jsonConf["brake_max"];
      }

      if (jsonConf.contains("brake_min"))
      {
         brake_limit_.lower = jsonConf["brake_min"];
      }

      if (jsonConf.contains("current_max"))
      {
         current_limit_.upper = jsonConf["current_max"];
      }

      if (jsonConf.contains("current_min"))
      {
         current_limit_.lower = jsonConf["current_min"];
      }

      if (jsonConf.contains("duty_cycle_max"))
      {
         duty_cycle_limit_.upper = jsonConf["duty_cycle_max"];
      }

      if (jsonConf.contains("duty_cycle_min"))
      {
         duty_cycle_limit_.lower = jsonConf["duty_cycle_min"];
      }

      if (jsonConf.contains("position_max"))
      {
         position_limit_.upper = jsonConf["position_max"];
      }

      if (jsonConf.contains("position_min"))
      {
         position_limit_.lower = jsonConf["position_min"];
      }

      if (jsonConf.contains("servo_max"))
      {
         position_limit_.upper = jsonConf["servo_max"];
      }

      if (jsonConf.contains("servo_min"))
      {
         position_limit_.lower = jsonConf["servo_min"];
      }

      if (jsonConf.contains("speed_max"))
      {
         speed_limit_.upper = jsonConf["speed_max"];
      }

      if (jsonConf.contains("speed_min"))
      {
         speed_limit_.lower = jsonConf["speed_min"];
      }
   }
   catch (std::exception& e)
   {
      std::cerr << "Exception: " << e.what() << std::endl;
      std::exit(EXIT_FAILURE);
   }

   configFile.close();
}

//This function will run every 20ms and run the timerCallback function
void VescDriver::runTimer(const std::chrono::milliseconds interval)
{
   while (timerThreadRunning_)
   {
      timerCallback();
      std::this_thread::sleep_for(std::chrono::milliseconds(interval));
   }
}

/* TODO or TO-THINKABOUT LIST
  - what should we do on startup? send brake or zero command?
  - what to do if the vesc interface gives an error?
  - check version number against know compatable?
  - should we wait until we receive telemetry before sending commands?
  - should we track the last motor command
  - what to do if no motor command received recently?
  - what to do if no servo command received recently?
  - what is the motor safe off state (0 current?)
  - what to do if a command parameter is out of range, ignore?
  - try to predict vesc bounds (from vesc config) and command detect bounds errors
*/

void VescDriver::timerCallback()
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected())
  {
    std::cout << "Unexpectedly disconnected from serial port.\n";
    std::exit(EXIT_FAILURE);
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING)
  {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0)
    {
      std::cout << "Connected to VESC with firmware version " << fw_version_major_ << "." << fw_version_minor_ << std::endl;
      driver_mode_ = MODE_OPERATING;
    }
  }
  else if (driver_mode_ == MODE_OPERATING)
  {
    // poll for vesc state (telemetry)
    vesc_.requestState();
    // poll for vesc imu
    vesc_.requestImuData();
  }
  else
  {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

int32_t VescDriver::getCurrentTimeInSeconds() const
{
   auto duration = std::chrono::system_clock::now().time_since_epoch();

   auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();

   if (seconds > std::numeric_limits<int32_t>::max())
   {
      std::cout << "ERROR: Time value exceeds int32_t range.\n";
      return 0;
   }

   return static_cast<int32_t>(seconds);
}

void VescDriver::vescPacketCallback(const std::shared_ptr<VescPacket const> & packet)
{
  if (packet->name() == "Values")
  {
    std::shared_ptr<VescPacketValues const> values =
      std::dynamic_pointer_cast<VescPacketValues const>(packet);

    auto state_msg = VescStateStamped();
    //state_msg.header.stamp = now();
    state_msg.header_().stamp().sec(getCurrentTimeInSeconds());
    //TODO do we also need to handle nanoseconds?

    state_msg.state().voltage_input(values->v_in());

    state_msg.state().current_input(values->avg_input_current());
    state_msg.state().current_motor(values->avg_motor_current());
    state_msg.state().avg_id(values->avg_id());
    state_msg.state().avg_iq(values->avg_iq());
    state_msg.state().duty_cycle(values->duty_cycle_now());
    state_msg.state().speed(values->rpm());

    state_msg.state().charge_drawn(values->amp_hours());
    state_msg.state().charge_regen(values->amp_hours_charged());
    state_msg.state().energy_drawn(values->watt_hours());
    state_msg.state().energy_regen(values->watt_hours_charged());
    state_msg.state().displacement(values->tachometer());
    state_msg.state().distance_traveled(values->tachometer_abs());
    state_msg.state().fault_code(static_cast<vesc_msgs::msg::FaultCode>(values->fault_code()));

    state_msg.state().pid_pos_now(values->pid_pos_now());
    state_msg.state().controller_id(values->controller_id());

    state_msg.state().ntc_temp_mos1(values->temp_mos1());
    state_msg.state().ntc_temp_mos2(values->temp_mos2());
    state_msg.state().ntc_temp_mos3(values->temp_mos3());
    state_msg.state().avg_vd(values->avg_vd());
    state_msg.state().avg_vq(values->avg_vq());

    //TODO initialize pubs and subs
    //state_pub_->publish(state_msg);
    state_pub_.write(state_msg);
  }
  else if (packet->name() == "FWVersion")
  {
    std::shared_ptr<VescPacketFWVersion const> fw_version =
      std::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
    std::cout << "-=" << fw_version->hwname() << "=- hardware paired " << fw_version->paired() << std::endl;
  }
  else if (packet->name() == "ImuData")
  {
    std::shared_ptr<VescPacketImu const> imuData =
      std::dynamic_pointer_cast<VescPacketImu const>(packet);

    auto imu_msg = VescImuStamped();
    auto std_imu_msg = Imu();
    imu_msg.header_().stamp().sec(getCurrentTimeInSeconds());
    std_imu_msg.header_().stamp().sec(getCurrentTimeInSeconds());

    imu_msg.imu_().ypr().x(imuData->roll());
    imu_msg.imu_().ypr().y(imuData->pitch());
    imu_msg.imu_().ypr().z(imuData->yaw());

    imu_msg.imu_().linear_acceleration().x(imuData->acc_x());
    imu_msg.imu_().linear_acceleration().y(imuData->acc_y());
    imu_msg.imu_().linear_acceleration().z(imuData->acc_z());

    imu_msg.imu_().angular_velocity().x(imuData->gyr_x());
    imu_msg.imu_().angular_velocity().y(imuData->gyr_y());
    imu_msg.imu_().angular_velocity().z(imuData->gyr_z());

    imu_msg.imu_().compass().x(imuData->mag_x());
    imu_msg.imu_().compass().y(imuData->mag_y());
    imu_msg.imu_().compass().z(imuData->mag_z());

    imu_msg.imu_().orientation().w(imuData->q_w());
    imu_msg.imu_().orientation().x(imuData->q_x());
    imu_msg.imu_().orientation().y(imuData->q_y());
    imu_msg.imu_().orientation().z(imuData->q_z());

    std_imu_msg.linear_acceleration().x(imuData->acc_x());
    std_imu_msg.linear_acceleration().y(imuData->acc_y());
    std_imu_msg.linear_acceleration().z(imuData->acc_z());

    std_imu_msg.angular_velocity().x(imuData->gyr_x());
    std_imu_msg.angular_velocity().y(imuData->gyr_y());
    std_imu_msg.angular_velocity().z(imuData->gyr_z());

    std_imu_msg.orientation().w(imuData->q_w());
    std_imu_msg.orientation().x(imuData->q_x());
    std_imu_msg.orientation().y(imuData->q_y());
    std_imu_msg.orientation().z(imuData->q_z());


    //imu_pub_->publish(imu_msg);
    //imu_std_pub_->publish(std_imu_msg);
    imu_pub_.write(imu_msg);
    imu_std_pub_.write(std_imu_msg);
  }
  /*auto & clk = *this->get_clock();
  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    clk,
    5000,
    "%s packet received",
    packet->name().c_str()
  );*/

  //TODO: Throttled debug message above
}

void VescDriver::vescErrorCallback(const std::string & error)
{
  std::cerr <<  error << std::endl;
}

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
//void VescDriver::dutyCycleCallback(const Float64 duty_cycle)
//{
//  if (driver_mode_ == MODE_OPERATING)
//  {
//    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle.data()));
//  }
//}

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
//void VescDriver::currentCallback(const Float64 current)
//{
//  if (driver_mode_ == MODE_OPERATING)
//  {
//    vesc_.setCurrent(current_limit_.clip(current.data()));
//  }
//}

/**
 * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
 *              However, note that the VESC may impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
//void VescDriver::brakeCallback(const Float64 brake)
//{
//  if (driver_mode_ == MODE_OPERATING)
//  {
//    vesc_.setBrake(brake_limit_.clip(brake.data()));
//  }
//}

/**
 * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
 *              multiplied by the number of motor poles. Any value is accepted by this
 *              driver. However, note that the VESC may impose a more restrictive bounds on the
 *              range depending on its configuration.
 */
void VescDriver::speedCallback(const Float64 speed)
{
  if (driver_mode_ == MODE_OPERATING)
  {
    vesc_.setSpeed(speed_limit_.clip(speed.data()));
  }
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
//void VescDriver::positionCallback(const Float64 position)
//{
//  if (driver_mode_ == MODE_OPERATING) {
//    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
//    double position_deg = position_limit_.clip(position.data()) * 180.0 / M_PI;
//    vesc_.setPosition(position_deg);
//  }
//}

/**
 * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
 */
//void VescDriver::servoCallback(const Float64 servo)
//{
//  if (driver_mode_ == MODE_OPERATING)
//  {
//    double servo_clipped(servo_limit_.clip(servo.data()));
//    vesc_.setServo(servo_clipped);
//    // publish clipped servo value as a "sensor"
//    auto servo_sensor_msg = Float64();
//    servo_sensor_msg.data(servo_clipped);
//    //servo_sensor_pub_->publish(servo_sensor_msg);
//    servo_sensor_pub_.write(servo_sensor_msg);
//  }
//}

VescDriver::CommandLimit::CommandLimit(
     const std::string & str,
     const std::optional<double> & min_lower,
     const std::optional<double> & max_upper
   )
   : name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  //auto param_min =
  //  node_ptr->declare_parameter(name + "_min", rclcpp::ParameterValue(0.0));
  //TODO get parameters from a config file and check for lower and upper values of
  //each CommandLimit

  /*if (min_lower.has_value())
  {
    if (min_lower && param_min.get<double>() < *min_lower)
    {
      lower = *min_lower;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_min (" << param_min.get<double>() <<
          ") is less than the feasible minimum (" << *min_lower << ").");
    } else if (max_upper && param_min.get<double>() > *max_upper) {
      lower = *max_upper;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_min (" << param_min.get<double>() <<
          ") is greater than the feasible maximum (" << *max_upper << ").");
    } else {
      lower = param_min.get<double>();
    }
  } else if (min_lower) {
    lower = *min_lower;
  }*/

  //// check if the uers' maximum value is outside of the range min_lower to max_upper
  //auto param_max =
  //  node_ptr->declare_parameter(name + "_max", rclcpp::ParameterValue(0.0));

  //if (param_max.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
  //  if (min_lower && param_max.get<double>() < *min_lower) {
  //    upper = *min_lower;
  //    RCLCPP_WARN_STREAM(
  //      logger, "Parameter " << name << "_max (" << param_max.get<double>() <<
  //        ") is less than the feasible minimum (" << *min_lower << ").");
  //  } else if (max_upper && param_max.get<double>() > *max_upper) {
  //    upper = *max_upper;
  //    RCLCPP_WARN_STREAM(
  //      logger, "Parameter " << name << "_max (" << param_max.get<double>() <<
  //        ") is greater than the feasible maximum (" << *max_upper << ").");
  //  } else {
  //    upper = param_max.get<double>();
  //  }
  //} else if (max_upper) {
  //  upper = *max_upper;
  //}

  //// check for min > max
  //if (upper && lower && *lower > *upper) {
  //  RCLCPP_WARN_STREAM(
  //    logger, "Parameter " << name << "_max (" << *upper <<
  //      ") is less than parameter " << name << "_min (" << *lower << ").");
  //  double temp(*lower);
  //  lower = *upper;
  //  upper = temp;
  //}

  std::ostringstream oss;
  oss << "  " << name << " limit: ";

  if (lower) {
    oss << *lower << " ";
  } else {
    oss << "(none) ";
  }

  if (upper) {
    oss << *upper;
  } else {
    oss << "(none)";
  }

  std::cout << oss.str() << std::endl;
}

double VescDriver::CommandLimit::clip(double value)
{
  //auto clock = rclcpp::Clock(RCL_ROS_TIME);
  //TODO throttle log message

  if (lower && value < lower) {
     std::cout << name << " command value (" << value << ") below minimum limit ("
        << *lower << "), clipping." << std::endl;
    return *lower;
  }
  if (upper && value > upper) {
     std::cout << name << " command value (" << value << ") above maximum limit ("
        << *upper << "), clipping." << std::endl;
    return *upper;
  }
  return value;
}

}  // namespace vesc_driver


int main(int argc, char** argv)
{
   if (argc != 2)
   {
      std::cerr << "Usage: " << argv[0] << " <vesc_config_file_absolute_path>\n";
      return 1;
   }

   const std::string configFilePath = argv[1];

   //TODO get params such as port and limits from a config file
   vesc_driver::VescDriver vd(configFilePath);
}

//#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

//RCLCPP_COMPONENTS_REGISTER_NODE(vesc_driver::VescDriver)

/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman */

#include <memory>
#include <string>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include "open_cr_module/open_cr_module.h"

namespace robotis_op
{

OpenCRModule::OpenCRModule()
    : control_cycle_msec_(8),
      DEBUG_PRINT(false),
      present_volt_(0.0),
      callback_node_(nullptr),
      callback_timer_(nullptr)
      
{
  module_name_ = "open_cr_module";  // set unique module name

  result_["gyro_x"] = 0.0;
  result_["gyro_y"] = 0.0;
  result_["gyro_z"] = 0.0;

  result_["acc_x"] = 0.0;
  result_["acc_y"] = 0.0;
  result_["acc_z"] = 0.0;

  result_["button_mode"] = 0;
  result_["button_start"] = 0;
  result_["button_user"] = 0;

  result_["present_voltage"] = 0.0;

  buttons_["button_mode"] = false;
  buttons_["button_start"] = false;
  buttons_["button_user"] = false;
  buttons_["published_mode"] = false;
  buttons_["published_start"] = false;
  buttons_["published_user"] = false;

  previous_result_["gyro_x"] = 0.0;
  previous_result_["gyro_y"] = 0.0;
  previous_result_["gyro_z"] = 0.0;

  previous_result_["gyro_x_prev"] = 0.0;
  previous_result_["gyro_y_prev"] = 0.0;
  previous_result_["gyro_z_prev"] = 0.0;

  previous_result_["acc_x"] = 0.0;
  previous_result_["acc_y"] = 0.0;
  previous_result_["acc_z"] = 0.0;

  last_msg_time_ = rclcpp::Clock().now();

  
}

OpenCRModule::~OpenCRModule()
{
  // No need to manually join the thread in ROS2
}

void OpenCRModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;

  

  // Create a separate node for handling callbacks
  callback_node_ = rclcpp::Node::make_shared("open_cr_module_callback");

  /* publisher */
  status_msg_pub_ = callback_node_->create_publisher<robotis_controller_msgs::msg::StatusMsg>("/robotis/status", 1);
  imu_pub_ = callback_node_->create_publisher<sensor_msgs::msg::Imu>("/robotis/open_cr/imu", 1);
  button_pub_ = callback_node_->create_publisher<std_msgs::msg::String>("/robotis/open_cr/button", 1);
  dxl_power_msg_pub_ = callback_node_->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 1);

  // Use a timer to process callbacks periodically

  auto process_callbacks =
    [this, &callback_node_ = callback_node_]() -> void
    {
      rclcpp::spin_some(callback_node_);
    };
  callback_timer_ = callback_node_->create_wall_timer(std::chrono::milliseconds(control_cycle_msec_), process_callbacks);
}


void OpenCRModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                           std::map<std::string, robotis_framework::Sensor *> sensors)
{
  if (sensors["open-cr"] == NULL)
    return;

  int16_t gyro_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_x"];
  int16_t gyro_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_y"];
  int16_t gyro_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_z"];

  int16_t acc_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_x"];
  int16_t acc_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_y"];
  int16_t acc_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_z"];

  uint16_t present_volt = sensors["open-cr"]->sensor_state_->bulk_read_table_["present_voltage"];

  result_["gyro_x"] = lowPassFilter(0.4, -getGyroValue(gyro_x), previous_result_["gyro_x"]);
  result_["gyro_y"] = lowPassFilter(0.4, -getGyroValue(gyro_y), previous_result_["gyro_y"]);
  result_["gyro_z"] = lowPassFilter(0.4, getGyroValue(gyro_z), previous_result_["gyro_z"]);

  // RCLCPP_INFO_EXPRESSION(callback_node_->get_logger(), DEBUG_PRINT, " ======================= Gyro ======================== ");
  // RCLCPP_INFO_EXPRESSION(callback_node_->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d", gyro_x, gyro_y, gyro_z);
  // RCLCPP_INFO_EXPRESSION(callback_node_->get_logger(), DEBUG_PRINT, "Filtered : %f, %f, %f", result_["gyro_x"], result_["gyro_y"], result_["gyro_z"]);

  // align axis of Accelerometer to robot and
  result_["acc_x"] = lowPassFilter(0.4, -getAccValue(acc_x), previous_result_["acc_x"]);
  result_["acc_y"] = lowPassFilter(0.4, -getAccValue(acc_y), previous_result_["acc_y"]);
  result_["acc_z"] = lowPassFilter(0.4, getAccValue(acc_z), previous_result_["acc_z"]);

  // RCLCPP_INFO_COND(callback_node_->get_logger(), DEBUG_PRINT, " ======================= Acc ======================== ");
  // RCLCPP_INFO_COND(callback_node_->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d", acc_x, acc_y, acc_z);
  // RCLCPP_INFO_COND(callback_node_->get_logger(), DEBUG_PRINT, "Filtered : %f, %f, %f", result_["acc_x"], result_["acc_y"], result_["acc_z"]);

  rclcpp::Time update_time(sensors["open-cr"]->sensor_state_->update_time_stamp_.sec_, sensors["open-cr"]->sensor_state_->update_time_stamp_.nsec_);
  rclcpp::Duration update_duration = rclcpp::Clock().now() - update_time;
  if (update_duration.nanoseconds() > 100000000)
    publishDXLPowerMsg(1);

  uint8_t button_flag = sensors["open-cr"]->sensor_state_->bulk_read_table_["button"];
  result_["button_mode"] = button_flag & 0x01;
  result_["button_start"] = (button_flag & 0x02) >> 1;
  result_["button_user"] = (button_flag & 0x04) >> 2;

  handleButton("mode");
  handleButton("start");
  handleButton("user");

  result_["present_voltage"] = present_volt * 0.1;
  handleVoltage(result_["present_voltage"]);

  publishIMU();

  previous_result_["gyro_x_prev"] = result_["gyro_x"];
  previous_result_["gyro_y_prev"] = result_["gyro_y"];
  previous_result_["gyro_z_prev"] = result_["gyro_z"];
}

// -2000 ~ 2000dps(-32800 ~ 32800), scale factor : 16.4, dps -> rps
double OpenCRModule::getGyroValue(int raw_value)
{
  return (double) raw_value * GYRO_FACTOR * DEGREE2RADIAN;
}

// -2.0 ~ 2.0g(-32768 ~ 32768), 1g = 9.8 m/s^2
double OpenCRModule::getAccValue(int raw_value)
{
  return (double) raw_value * ACCEL_FACTOR;
}

void OpenCRModule::publishIMU()
{
  // fusion imu data
  imu_msg_.header.stamp = rclcpp::Clock().now();
  imu_msg_.header.frame_id = "body_link";
  double filter_alpha = 0.4;

  //in rad/s
  long int _value = 0;
  int _arrd_length = 2;

  imu_msg_.angular_velocity.x = result_["gyro_x"];
  imu_msg_.angular_velocity.y = result_["gyro_y"];
  imu_msg_.angular_velocity.z = result_["gyro_z"];

  //in m/s^2
  imu_msg_.linear_acceleration.x = result_["acc_x"] * G_ACC;
  imu_msg_.linear_acceleration.y = result_["acc_y"] * G_ACC;
  imu_msg_.linear_acceleration.z = result_["acc_z"] * G_ACC;

  //Estimation of roll and pitch based on accelometer data, see http://www.nxp.com/files/sensors/doc/app_note/AN3461.pdf
  double mui = 0.01;
  double sign = copysignf(1.0, result_["acc_z"]);
  double roll = atan2(result_["acc_y"],
                      sign * sqrt(result_["acc_z"] * result_["acc_z"] + mui * result_["acc_x"] * result_["acc_x"]));
  double pitch = atan2(-result_["acc_x"],
                       sqrt(result_["acc_y"] * result_["acc_y"] + result_["acc_z"] * result_["acc_z"]));
  double yaw = 0.0;

  Eigen::Quaterniond orientation = robotis_framework::convertRPYToQuaternion(roll, pitch, yaw);

  imu_msg_.orientation.x = orientation.x();
  imu_msg_.orientation.y = orientation.y();
  imu_msg_.orientation.z = orientation.z();
  imu_msg_.orientation.w = orientation.w();

  imu_pub_->publish(imu_msg_);
}

void OpenCRModule::handleButton(const std::string &button_name)
{
  std::string button_key = "button_" + button_name;
  std::string button_published = "published_" + button_name;

  bool pushed = (result_[button_key] == 1.0);
  // same state
  if (buttons_[button_key] == pushed)
  {
    if (pushed == true && buttons_[button_published] == false)
    {
      // check long press
      rclcpp::Duration button_duration = rclcpp::Clock().now() - buttons_press_time_[button_name];
      if (button_duration.seconds() > 2.0)
      {
        publishButtonMsg(button_name + "_long");
        buttons_[button_published] = true;
      }
    }
  }
  else    // state is changed
  {
    buttons_[button_key] = pushed;

    if (pushed == true)
    {
      buttons_press_time_[button_name] = rclcpp::Clock().now();
      buttons_[button_published] = false;
    }
    else
    {
      rclcpp::Duration button_duration = rclcpp::Clock().now() - buttons_press_time_[button_name];

      if (button_duration.seconds() < 2.0)     // short press
        publishButtonMsg(button_name);
      else
        // long press
        ;
    }
  }
}

void OpenCRModule::publishButtonMsg(const std::string &button_name)
{
  std_msgs::msg::String button_msg;
  button_msg.data = button_name;

  button_pub_->publish(button_msg);
  publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Button : " + button_name);
}

void OpenCRModule::handleVoltage(double present_volt)
{
  double voltage_ratio = 0.4;
  previous_volt_ = (previous_volt_ != 0) ? previous_volt_ * (1 - voltage_ratio) + present_volt * voltage_ratio : present_volt;

  if (fabs(present_volt_ - previous_volt_) >= 0.1)
  {
    // check last publised time
    rclcpp::Time now = rclcpp::Clock().now();
    rclcpp::Duration dur = now - last_msg_time_;
    if (dur.seconds() < 1)
      return;

    last_msg_time_ = now;

    present_volt_ = previous_volt_;
    std::stringstream log_stream;
    log_stream << "Present Volt : " << present_volt_ << "V";
    publishStatusMsg((present_volt_ < 11 ? robotis_controller_msgs::msg::StatusMsg::STATUS_WARN : robotis_controller_msgs::msg::StatusMsg::STATUS_INFO),
                     log_stream.str());
    // RCLCPP_INFO_EXPRESSION(callback_node_->get_logger(), static_cast<bool>(DEBUG_PRINT), "Present Volt : %fV, Read Volt : %fV", previous_volt_, result_["present_voltage"]);
  }
}

void OpenCRModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::msg::StatusMsg status_msg;
  status_msg.header.stamp = rclcpp::Clock().now();
  status_msg.type = type;
  status_msg.module_name = "SENSOR";
  status_msg.status_msg = msg;

  status_msg_pub_->publish(status_msg);
}

void OpenCRModule::publishDXLPowerMsg(unsigned int value)
{
  robotis_controller_msgs::msg::SyncWriteItem sync_write_msg;
  sync_write_msg.item_name = "dynamixel_power";
  sync_write_msg.joint_name.push_back("open-cr");
  sync_write_msg.value.push_back(value);

  dxl_power_msg_pub_->publish(sync_write_msg);
}

double OpenCRModule::lowPassFilter(double alpha, double x_new, double &x_old)
{
  double filtered_value = alpha * x_new + (1.0 - alpha) * x_old;
  x_old = filtered_value;

  return filtered_value;
}

}

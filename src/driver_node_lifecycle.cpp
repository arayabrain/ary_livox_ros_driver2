//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "driver_node_lifecycle.h"

#ifdef BUILDING_ROS2

#include <chrono>

#include "include/livox_ros_driver2.h"
#include "lddc.h"
#include "lds_lidar.h"

namespace livox_ros {

DriverNodeLifecycle::DriverNodeLifecycle(const rclcpp::NodeOptions& options)
    : LifecycleNode("livox_driver_node", options) {
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s (lifecycle)",
              LIVOX_ROS_DRIVER2_VERSION_STRING);

  // Same parameter set as DriverNode, plus activate_timeout_ms. Declared here,
  // read in on_configure.
  this->declare_parameter("xfer_format", static_cast<int>(kPointCloud2Msg));
  this->declare_parameter("multi_topic", 0);
  this->declare_parameter("data_src", static_cast<int>(kSourceRawLidar));
  this->declare_parameter("publish_freq", 10.0);
  this->declare_parameter("output_data_type", static_cast<int>(kOutputToRos));
  this->declare_parameter("frame_id", "frame_default");
  this->declare_parameter("user_config_path", "path_default");
  this->declare_parameter("cmdline_input_bd_code", "000000000000001");
  this->declare_parameter("lvx_file_path", "/home/livox/livox_test.lvx");
  this->declare_parameter("sleep_on_shutdown", true);
  this->declare_parameter("activate_timeout_ms", 5000);
}

DriverNodeLifecycle::~DriverNodeLifecycle() { TeardownDriver(); }

DriverNodeLifecycle::CallbackReturn DriverNodeLifecycle::on_configure(
    const rclcpp_lifecycle::State& /*state*/) {
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id;

  this->get_parameter("xfer_format", xfer_format);
  this->get_parameter("multi_topic", multi_topic);
  this->get_parameter("data_src", data_src);
  this->get_parameter("publish_freq", publish_freq);
  this->get_parameter("output_data_type", output_type);
  this->get_parameter("frame_id", frame_id);
  this->get_parameter("sleep_on_shutdown", sleep_on_shutdown_);
  this->get_parameter("activate_timeout_ms", activate_timeout_ms_);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  }

  if (data_src != kSourceRawLidar) {
    DRIVER_ERROR(*this, "Invalid data src (%d), please check the launch file", data_src);
    return CallbackReturn::FAILURE;
  }

  exit_signal_ = std::promise<void>();
  future_ = exit_signal_.get_future();

  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src,
                                     output_type, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(this);

  std::string user_config_path;
  this->get_parameter("user_config_path", user_config_path);
  DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

  lds_lidar_ = LdsLidar::GetInstance(publish_freq);
  // Suppress the discovery callback's automatic motor-on BEFORE the SDK comes
  // up; the LiDAR stays (or is put) in standby until on_activate.
  lds_lidar_->SetWakeOnConnect(false);
  lddc_ptr_->RegisterLds(static_cast<Lds*>(lds_lidar_));

  if (!lds_lidar_->InitLdsLidar(user_config_path)) {
    DRIVER_ERROR(*this, "Init lds lidar fail!");
    lddc_ptr_.reset();
    lds_lidar_ = nullptr;
    return CallbackReturn::FAILURE;
  }
  DRIVER_INFO(*this, "Init lds lidar success!");

  // The poll threads just block on the data semaphore while the motor is off,
  // so it is safe (and simplest) to run them from configure onwards.
  pointclouddata_poll_thread_ = std::make_shared<std::thread>(
      &DriverNodeLifecycle::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(
      &DriverNodeLifecycle::ImuDataPollThread, this);

  driver_running_ = true;
  DRIVER_INFO(*this, "Configured. LiDAR is in STANDBY (motor off).");
  return CallbackReturn::SUCCESS;
}

DriverNodeLifecycle::CallbackReturn DriverNodeLifecycle::on_activate(
    const rclcpp_lifecycle::State& /*state*/) {
  if (lds_lidar_ == nullptr) {
    return CallbackReturn::FAILURE;
  }
  // From now on a LiDAR that (re)connects is woken automatically.
  lds_lidar_->SetWakeOnConnect(true);
  if (!lds_lidar_->WakeAllLidarsBlocking(
          std::chrono::milliseconds(activate_timeout_ms_))) {
    DRIVER_ERROR(*this,
                 "Failed to wake LiDAR(s) within %d ms - is the LiDAR "
                 "connected and powered?",
                 activate_timeout_ms_);
    lds_lidar_->SetWakeOnConnect(false);
    return CallbackReturn::FAILURE;  // rolls back to 'inactive'
  }
  DRIVER_INFO(*this, "Activated. LiDAR is SCANNING (motor on).");
  return CallbackReturn::SUCCESS;
}

DriverNodeLifecycle::CallbackReturn DriverNodeLifecycle::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/) {
  if (lds_lidar_ != nullptr) {
    lds_lidar_->SetWakeOnConnect(false);
    lds_lidar_->SleepAllLidarsBlocking(std::chrono::milliseconds(2000));
  }
  DRIVER_INFO(*this, "Deactivated. LiDAR is in STANDBY (motor off).");
  return CallbackReturn::SUCCESS;  // best effort - always succeed
}

DriverNodeLifecycle::CallbackReturn DriverNodeLifecycle::on_cleanup(
    const rclcpp_lifecycle::State& /*state*/) {
  TeardownDriver();
  return CallbackReturn::SUCCESS;
}

DriverNodeLifecycle::CallbackReturn DriverNodeLifecycle::on_shutdown(
    const rclcpp_lifecycle::State& /*state*/) {
  TeardownDriver();
  return CallbackReturn::SUCCESS;
}

void DriverNodeLifecycle::TeardownDriver() {
  if (!driver_running_) {
    return;
  }
  driver_running_ = false;

  // Same sequence as DriverNode::~DriverNode (driver_node.cpp).
  if (sleep_on_shutdown_ && lddc_ptr_ && lddc_ptr_->lds_) {
    static_cast<LdsLidar*>(lddc_ptr_->lds_)
        ->SleepAllLidarsBlocking(std::chrono::milliseconds(2000));
  }
  if (lddc_ptr_ && lddc_ptr_->lds_) {
    lddc_ptr_->lds_->RequestExit();
  }
  exit_signal_.set_value();
  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
    pointclouddata_poll_thread_->join();
  }
  if (imudata_poll_thread_ && imudata_poll_thread_->joinable()) {
    imudata_poll_thread_->join();
  }
  if (lddc_ptr_ && lddc_ptr_->lds_) {
    lddc_ptr_->lds_->PrepareExit();
  }
  lddc_ptr_.reset();
  lds_lidar_ = nullptr;
}

void DriverNodeLifecycle::PointCloudDataPollThread() {
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNodeLifecycle::ImuDataPollThread() {
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNodeLifecycle)

#endif  // BUILDING_ROS2

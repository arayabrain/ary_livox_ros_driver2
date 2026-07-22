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

#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>

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
  // Sensors (JSON config "name" fields) that on_activate wakes and that
  // ~/set_sensor_mode refuses to put in standby. All other configured sensors
  // stay in standby until woken through the service.
  this->declare_parameter("always_on_sensors",
                          std::vector<std::string>{"center"});
  this->declare_parameter("mode_switch_timeout_ms", 2000);
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

  // Validate the always-on set against the configured sensor names; unknown
  // entries are dropped (with a warning) so a typo cannot make on_activate
  // wait on a sensor that does not exist.
  this->get_parameter("always_on_sensors", always_on_sensors_);
  this->get_parameter("mode_switch_timeout_ms", mode_switch_timeout_ms_);
  const auto configured = lds_lidar_->GetConfiguredSensorNames();
  std::vector<std::string> valid;
  for (const auto& name : always_on_sensors_) {
    if (std::find(configured.begin(), configured.end(), name) !=
        configured.end()) {
      valid.push_back(name);
    } else {
      DRIVER_WARN(*this,
                  "always_on_sensors entry '%s' is not in the LiDAR config - "
                  "ignoring it",
                  name.c_str());
    }
  }
  always_on_sensors_ = valid;
  if (always_on_sensors_.empty()) {
    DRIVER_WARN(*this,
                "always_on_sensors is empty: on_activate will not wake any "
                "sensor; use ~/set_sensor_mode to wake sensors on demand.");
  }

  set_sensor_mode_srv_ =
      this->create_service<livox_ros_driver2::srv::SetSensorMode>(
          "~/set_sensor_mode",
          std::bind(&DriverNodeLifecycle::HandleSetSensorMode, this,
                    std::placeholders::_1, std::placeholders::_2));

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
  if (always_on_sensors_.empty()) {
    DRIVER_WARN(*this,
                "Activated with an empty always-on set: every sensor stays in "
                "STANDBY until woken via ~/set_sensor_mode.");
    return CallbackReturn::SUCCESS;
  }
  // Wake only the always-on set; the wake is recorded as each handle's
  // desired mode, so a sensor that reconnects (power blip) wakes again while
  // every other sensor keeps standby as its connect-time mode.
  std::vector<LdsLidar::SensorModeResult> results;
  if (!lds_lidar_->SetLidarsWorkModeBlocking(
          always_on_sensors_, kLivoxLidarNormal,
          std::chrono::milliseconds(activate_timeout_ms_), &results)) {
    for (const auto& r : results) {
      if (!r.ok) {
        DRIVER_ERROR(*this, "always-on sensor '%s' failed to wake: %s",
                     r.name.c_str(), r.detail.c_str());
      }
    }
    DRIVER_ERROR(*this,
                 "Failed to wake always-on LiDAR(s) within %d ms - is the "
                 "LiDAR connected and powered?",
                 activate_timeout_ms_);
    // Roll back: clear the wake intent and best-effort sleep anything woken.
    lds_lidar_->SetDefaultModeOnConnect(kLivoxLidarWakeUp);
    lds_lidar_->SetLidarsWorkModeBlocking(always_on_sensors_,
                                          kLivoxLidarWakeUp,
                                          std::chrono::milliseconds(2000));
    return CallbackReturn::FAILURE;  // rolls back to 'inactive'
  }
  DRIVER_INFO(*this,
              "Activated. Always-on sensor(s) SCANNING (motor on); remaining "
              "sensors in STANDBY - wake them via ~/set_sensor_mode.");
  return CallbackReturn::SUCCESS;
}

DriverNodeLifecycle::CallbackReturn DriverNodeLifecycle::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/) {
  if (lds_lidar_ != nullptr) {
    // Reset every per-sensor wake intent, then best-effort sleep everything.
    lds_lidar_->SetDefaultModeOnConnect(kLivoxLidarWakeUp);
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

void DriverNodeLifecycle::HandleSetSensorMode(
    const std::shared_ptr<livox_ros_driver2::srv::SetSensorMode::Request> req,
    std::shared_ptr<livox_ros_driver2::srv::SetSensorMode::Response> res) {
  auto refuse_all = [&](const std::string& why) {
    res->success = false;
    res->message = why;
    for (const auto& name : req->sensors) {
      res->sensors.push_back(name);
      res->ok.push_back(false);
      res->detail.push_back(why);
    }
  };

  if (lds_lidar_ == nullptr) {
    refuse_all("driver not configured");
    return;
  }

  // Normalize: empty or ["all"] means every configured sensor.
  std::vector<std::string> names = req->sensors;
  if (names.size() == 1 && names.front() == "all") {
    names.clear();
  }
  if (names.empty()) {
    names = lds_lidar_->GetConfiguredSensorNames();
  }

  const bool node_active =
      this->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  if (req->active && !node_active) {
    refuse_all("node is not ACTIVE; wake refused");
    return;
  }

  // Standby requests must not silence the always-on set (the LIO feed):
  // record a per-sensor refusal and process the rest.
  std::vector<LdsLidar::SensorModeResult> results;
  std::vector<std::string> to_send;
  for (const auto& name : names) {
    if (!req->active &&
        std::find(always_on_sensors_.begin(), always_on_sensors_.end(),
                  name) != always_on_sensors_.end()) {
      results.push_back(
          {name, false, "always-on sensor; deactivate the node instead"});
    } else {
      to_send.push_back(name);
    }
  }

  const auto timeout = std::chrono::milliseconds(
      req->timeout_ms > 0 ? static_cast<int>(req->timeout_ms)
                          : mode_switch_timeout_ms_);
  if (!to_send.empty()) {
    lds_lidar_->SetLidarsWorkModeBlocking(
        to_send, req->active ? kLivoxLidarNormal : kLivoxLidarWakeUp, timeout,
        &results);
  }

  size_t ok_count = 0;
  for (const auto& r : results) {
    res->sensors.push_back(r.name);
    res->ok.push_back(r.ok);
    res->detail.push_back(r.detail);
    if (r.ok) {
      ++ok_count;
    }
  }
  res->success = ok_count == results.size() && !results.empty();
  std::ostringstream msg;
  msg << ok_count << "/" << results.size() << " ok ("
      << (req->active ? "wake" : "standby") << ")";
  res->message = msg.str();
  DRIVER_INFO(*this, "set_sensor_mode: %s", res->message.c_str());
}

void DriverNodeLifecycle::TeardownDriver() {
  if (!driver_running_) {
    return;
  }
  driver_running_ = false;
  set_sensor_mode_srv_.reset();

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

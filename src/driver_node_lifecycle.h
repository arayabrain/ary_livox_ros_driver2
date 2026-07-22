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

#ifndef LIVOX_DRIVER_NODE_LIFECYCLE_H
#define LIVOX_DRIVER_NODE_LIFECYCLE_H

#include "include/ros_headers.h"

#ifdef BUILDING_ROS2

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <string>
#include <vector>

#include "livox_ros_driver2/srv/set_sensor_mode.hpp"

namespace livox_ros {

class Lddc;
class LdsLidar;

/**
 * Lifecycle variant of DriverNode (ROS 2 only). The regular DriverNode is
 * untouched; this node exists so the LiDAR motor can be toggled at runtime:
 *
 *   on_configure   - read params, init the Livox SDK, discover the LiDAR.
 *                    The LiDAR is kept (or put) in standby: motor OFF.
 *   on_activate    - kLivoxLidarNormal on all LiDARs: motor ON, data flows.
 *   on_deactivate  - kLivoxLidarWakeUp (MID-360 standby): motor OFF.
 *   on_cleanup /
 *   on_shutdown    - full teardown, same sequence as ~DriverNode.
 *
 * The SDK stays alive across activate/deactivate cycles; only the LiDAR work
 * mode changes. configure -> cleanup -> configure is best effort only (the
 * Livox SDK does not guarantee re-init after uninit within one process).
 *
 * Per-sensor control (multi-sensor configs): on_activate only wakes the
 * sensors listed in the `always_on_sensors` param (default ["center"]); the
 * remaining sensors stay in standby and are woken/slept on demand through the
 * ~/set_sensor_mode service (livox_ros_driver2/srv/SetSensorMode). Wake
 * requests are refused unless the node is ACTIVE; standby requests for
 * always-on sensors are refused (deactivate the node instead).
 */
class DriverNodeLifecycle : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit DriverNodeLifecycle(const rclcpp::NodeOptions& options);
  DriverNodeLifecycle(const DriverNodeLifecycle&) = delete;
  ~DriverNodeLifecycle() override;
  DriverNodeLifecycle& operator=(const DriverNodeLifecycle&) = delete;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

 private:
  void PointCloudDataPollThread();
  void ImuDataPollThread();
  void TeardownDriver();  // idempotent ~DriverNode-equivalent sequence

  void HandleSetSensorMode(
      const std::shared_ptr<livox_ros_driver2::srv::SetSensorMode::Request> req,
      std::shared_ptr<livox_ros_driver2::srv::SetSensorMode::Response> res);

  std::unique_ptr<Lddc> lddc_ptr_;
  LdsLidar* lds_lidar_ = nullptr;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;
  rclcpp::Service<livox_ros_driver2::srv::SetSensorMode>::SharedPtr
      set_sensor_mode_srv_;
  std::vector<std::string> always_on_sensors_;
  bool sleep_on_shutdown_ = true;
  int activate_timeout_ms_ = 5000;
  int mode_switch_timeout_ms_ = 2000;
  bool driver_running_ = false;
};

}  // namespace livox_ros

#endif  // BUILDING_ROS2

#endif  // LIVOX_DRIVER_NODE_LIFECYCLE_H

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

/** Livox LiDAR data source, data from dependent lidar */

#ifndef LIVOX_ROS_DRIVER_LDS_LIDAR_H_
#define LIVOX_ROS_DRIVER_LDS_LIDAR_H_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "lds.h"
#include "comm/comm.h"

#include "livox_lidar_def.h"

#include "rapidjson/document.h"

namespace livox_ros {

class LdsLidar final : public Lds {
 public:
  static LdsLidar *GetInstance(double publish_freq) {
    printf("LdsLidar *GetInstance\n");
    static LdsLidar lds_lidar(publish_freq);
    return &lds_lidar;
  }

  bool InitLdsLidar(const std::string& path_name);
  bool Start();

  int DeInitLdsLidar(void);

  /**
   * Send kLivoxLidarWakeUp (MID-360 low-power standby) to every connected
   * LiDAR and block until all acknowledge, or the timeout elapses. Safe to
   * call from the driver node destructor: it does not retry and always
   * returns within `timeout`. NOTE: despite the enum name, kLivoxLidarWakeUp
   * is the correct "sleep / motor off" mode on MID-360 — see Livox-SDK2#103.
   */
  void SleepAllLidarsBlocking(std::chrono::milliseconds timeout);

  /**
   * Send kLivoxLidarNormal (motor on, scanning) to every configured LiDAR and
   * block until all acknowledge. Returns false when no LiDAR is configured or
   * the acks do not arrive within `timeout` (e.g. LiDAR unplugged).
   */
  bool WakeAllLidarsBlocking(std::chrono::milliseconds timeout);

  /** Per-sensor outcome of SetLidarsWorkModeBlocking. */
  struct SensorModeResult {
    std::string name;
    bool ok = false;
    std::string detail;  // "" on success
  };

  /**
   * Send `mode` to the sensors named in `names` (JSON config "name" fields;
   * empty = every configured sensor) and block until each acknowledges or
   * `timeout` elapses. Records the requested mode as the per-handle desired
   * mode BEFORE sending, so a sensor that reconnects later (power blip) is
   * restored to what the caller last asked for. Per-sensor outcomes land in
   * `results` (nullable); returns true iff every resolved sensor succeeded.
   * Unknown names and disconnected sensors fail immediately without waiting.
   */
  bool SetLidarsWorkModeBlocking(const std::vector<std::string>& names,
                                 LivoxLidarWorkMode mode,
                                 std::chrono::milliseconds timeout,
                                 std::vector<SensorModeResult>* results = nullptr);

  /** Configured sensor names, in config order (valid after InitLdsLidar). */
  std::vector<std::string> GetConfiguredSensorNames() const;

  /**
   * Work mode the discovery callback applies when a LiDAR (re)connects:
   * the handle's recorded desired mode if one was set (via
   * SetLidarsWorkModeBlocking), else the default-on-connect mode.
   */
  LivoxLidarWorkMode DesiredModeOnConnect(uint32_t handle) const;

  /**
   * Sets the default connect-time mode AND resets every per-handle desired
   * mode to it (a lifecycle transition is a global statement of intent):
   * kLivoxLidarNormal -> motor on when a sensor connects (plain driver
   * behavior); kLivoxLidarWakeUp -> standby until asked otherwise (lifecycle
   * driver while not ACTIVE).
   */
  void SetDefaultModeOnConnect(LivoxLidarWorkMode mode);

  /**
   * Compatibility shim for the plain DriverNode / livox_set_mode tool and the
   * existing lifecycle transitions: maps true/false onto
   * SetDefaultModeOnConnect(kLivoxLidarNormal / kLivoxLidarWakeUp).
   */
  void SetWakeOnConnect(bool enable) {
    SetDefaultModeOnConnect(enable ? kLivoxLidarNormal : kLivoxLidarWakeUp);
  }
  bool IsWakeOnConnect() const {
    return default_mode_on_connect_.load() == kLivoxLidarNormal;
  }

 private:
  LdsLidar(double publish_freq);
  LdsLidar(const LdsLidar &) = delete;
  ~LdsLidar();
  LdsLidar &operator=(const LdsLidar &) = delete;

  bool ParseSummaryConfig();

  /** Resolve config names to (name, handle) pairs; unresolved names get an
   *  immediate failed SensorModeResult appended to `failed`. Empty `names`
   *  resolves to every configured sensor. */
  std::vector<std::pair<std::string, uint32_t>> ResolveSensorNames(
      const std::vector<std::string>& names,
      std::vector<SensorModeResult>* failed) const;

  bool InitLidars();
  bool InitLivoxLidar();    // for new SDK

  bool LivoxLidarStart();

  void ResetLdsLidar(void);

  void SetLidarPubHandle();

	// auto connect mode
	void EnableAutoConnectMode(void) { auto_connect_mode_ = true; }
  void DisableAutoConnectMode(void) { auto_connect_mode_ = false; }
  bool IsAutoConnectMode(void) { return auto_connect_mode_; }

  virtual void PrepareExit(void);

 public:
  std::mutex config_mutex_;

 private:
  std::string path_;
  LidarSummaryInfo lidar_summary_info_;

  bool auto_connect_mode_;
  uint32_t whitelist_count_;
  volatile bool is_initialized_;
  // Connect-time work-mode policy: per-handle overrides (last explicit
  // request via SetLidarsWorkModeBlocking) on top of a global default.
  std::atomic<LivoxLidarWorkMode> default_mode_on_connect_{kLivoxLidarNormal};
  mutable std::mutex desired_mode_mutex_;
  std::unordered_map<uint32_t, LivoxLidarWorkMode> desired_mode_;
  char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize];
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER_LDS_LIDAR_H_

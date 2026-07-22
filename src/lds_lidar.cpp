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

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>

#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif // WIN32
#include "livox_lidar_api.h"
#include "comm/comm.h"
#include "comm/pub_handler.h"

#include "parse_cfg_file/parse_cfg_file.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

#include "call_back/lidar_common_callback.h"
#include "call_back/livox_lidar_callback.h"

using namespace std;

namespace livox_ros {

/** Const varible ------------------------------------------------------------*/
/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

/** Global function for common use -------------------------------------------*/

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(double publish_freq)
    : Lds(publish_freq, kSourceRawLidar), 
      auto_connect_mode_(true),
      whitelist_count_(0),
      is_initialized_(false) {
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }



bool LdsLidar::InitLdsLidar(const std::string& path_name) {
  if (is_initialized_) {
    printf("Lds is already inited!\n");
    return false;
  }

  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;
  }

  path_ = path_name;
  if (!InitLidars()) {
    return false;
  }
  SetLidarPubHandle();
  if (!Start()) {
    return false;
  }
  is_initialized_ = true;
  return true;
}

bool LdsLidar::InitLidars() {
  if (!ParseSummaryConfig()) {
    return false;
  }
  std::cout << "config lidar type: " << static_cast<int>(lidar_summary_info_.lidar_type) << std::endl;

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!InitLivoxLidar()) {
      return false;
    }
  }
  return true;
}


bool LdsLidar::Start() {
  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!LivoxLidarStart()) {
      return false;
    }
  }
  return true;
}

bool LdsLidar::ParseSummaryConfig() {
  return ParseCfgFile(path_).ParseSummaryInfo(lidar_summary_info_);
}

bool LdsLidar::InitLivoxLidar() {
#ifdef BUILDING_ROS2
  DisableLivoxSdkConsoleLogger();
#endif

  // parse user config
  LivoxLidarConfigParser parser(path_);
  std::vector<UserLivoxLidarConfig> user_configs;
  if (!parser.Parse(user_configs)) {
    std::cout << "failed to parse user-defined config" << std::endl;
  }

  // SDK initialization
  if (!LivoxLidarSdkInit(path_.c_str())) {
    std::cout << "Failed to init livox lidar sdk." << std::endl;
    return false;
  }

  // fill in lidar devices
  for (auto& config : user_configs) {
    uint8_t index = 0;
    int8_t ret = g_lds_ldiar->cache_index_.GetFreeIndex(kLivoxLidarType, config.handle, index);
    if (ret != 0) {
      std::cout << "failed to get free index, lidar ip: " << IpNumToString(config.handle) << std::endl;
      continue;
    }
    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
    p_lidar->livox_config = config;
    p_lidar->handle = config.handle;

    LidarExtParameter lidar_param;
    lidar_param.handle = config.handle;
    lidar_param.lidar_type = kLivoxLidarType;
    if (config.pcl_data_type == kLivoxLidarCartesianCoordinateLowData) {
      // temporary resolution
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x / 10;
      lidar_param.param.y     = config.extrinsic_param.y / 10;
      lidar_param.param.z     = config.extrinsic_param.z / 10;
    } else {
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x;
      lidar_param.param.y     = config.extrinsic_param.y;
      lidar_param.param.z     = config.extrinsic_param.z;
    }
    pub_handler().AddLidarsExtParam(lidar_param);
  }

  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);
  return true;
}

void LdsLidar::SetLidarPubHandle() {
  pub_handler().SetPointCloudsCallback(LidarCommonCallback::OnLidarPointClounCb, g_lds_ldiar);
  pub_handler().SetImuDataCallback(LidarCommonCallback::LidarImuDataCallback, g_lds_ldiar);

  double publish_freq = Lds::GetLdsFrequency();
  pub_handler().SetPointCloudConfig(publish_freq);
}

bool LdsLidar::LivoxLidarStart() {
  return true;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    LivoxLidarSdkUninit();
    printf("Livox Lidar SDK Deinit completely!\n");
  }

  // Allow a later re-init (lifecycle cleanup -> configure). Best effort: the
  // Livox SDK does not guarantee init after uninit within one process.
  is_initialized_ = false;
  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

namespace {

// Per-handle ack collection for SetLidarsWorkModeBlocking. Heap-allocated and
// reference-counted by hand: when the blocking caller gives up (timeout) it
// marks the state `abandoned`; the last late ack callback then frees it. This
// avoids the use-after-free a stack-allocated state would have when an ack
// arrives after the caller returned (expected with unplugged sensors).
struct WorkModeAckState {
  std::mutex mu;
  std::condition_variable cv;
  std::unordered_map<uint32_t, livox_status> done;  // handle -> ack status
  uint32_t outstanding = 0;  // commands sent, ack still pending
  bool abandoned = false;    // caller returned; last callback deletes
};

void WorkModeAckCallback(livox_status status, uint32_t handle,
                         LivoxLidarAsyncControlResponse* /*response*/,
                         void* client_data) {
  auto* state = static_cast<WorkModeAckState*>(client_data);
  if (state == nullptr) return;
  if (status != kLivoxLidarStatusSuccess) {
    std::cout << "work-mode command failed, handle: " << handle
              << ", status: " << status << std::endl;
  } else {
    std::cout << "work-mode acknowledged, handle: " << handle << std::endl;
  }
  bool destroy = false;
  {
    std::lock_guard<std::mutex> lock(state->mu);
    state->done[handle] = status;
    if (state->outstanding > 0) {
      --state->outstanding;
    }
    destroy = state->abandoned && state->outstanding == 0;
    // Notify while holding the lock: the waiting caller cannot resume (and
    // possibly delete the state) until this callback releases the mutex.
    state->cv.notify_all();
  }
  if (destroy) {
    delete state;
  }
}

}  // namespace

std::vector<std::string> LdsLidar::GetConfiguredSensorNames() const {
  std::vector<std::string> names;
  for (uint8_t i = 0; i < kMaxSourceLidar; ++i) {
    if (lidars_[i].handle != 0 && lidars_[i].lidar_type == kLivoxLidarType) {
      names.push_back(lidars_[i].livox_config.topic_name);
    }
  }
  return names;
}

std::vector<std::pair<std::string, uint32_t>> LdsLidar::ResolveSensorNames(
    const std::vector<std::string>& names,
    std::vector<SensorModeResult>* failed) const {
  std::vector<std::pair<std::string, uint32_t>> resolved;
  if (names.empty()) {
    for (uint8_t i = 0; i < kMaxSourceLidar; ++i) {
      if (lidars_[i].handle != 0 && lidars_[i].lidar_type == kLivoxLidarType) {
        resolved.emplace_back(lidars_[i].livox_config.topic_name,
                              lidars_[i].handle);
      }
    }
    return resolved;
  }
  for (const auto& name : names) {
    uint32_t handle = 0;
    for (uint8_t i = 0; i < kMaxSourceLidar; ++i) {
      if (lidars_[i].handle != 0 && lidars_[i].lidar_type == kLivoxLidarType &&
          lidars_[i].livox_config.topic_name == name) {
        handle = lidars_[i].handle;
        break;
      }
    }
    if (handle != 0) {
      resolved.emplace_back(name, handle);
    } else if (failed != nullptr) {
      failed->push_back({name, false, "unknown sensor"});
    }
  }
  return resolved;
}

LivoxLidarWorkMode LdsLidar::DesiredModeOnConnect(uint32_t handle) const {
  std::lock_guard<std::mutex> lock(desired_mode_mutex_);
  auto it = desired_mode_.find(handle);
  if (it != desired_mode_.end()) {
    return it->second;
  }
  return default_mode_on_connect_.load();
}

void LdsLidar::SetDefaultModeOnConnect(LivoxLidarWorkMode mode) {
  default_mode_on_connect_.store(mode);
  std::lock_guard<std::mutex> lock(desired_mode_mutex_);
  desired_mode_.clear();
}

bool LdsLidar::SetLidarsWorkModeBlocking(
    const std::vector<std::string>& names, LivoxLidarWorkMode mode,
    std::chrono::milliseconds timeout,
    std::vector<SensorModeResult>* results) {
  std::vector<SensorModeResult> local;
  std::vector<SensorModeResult>& out = results ? *results : local;

  if (!is_initialized_) {
    for (const auto& name : names) {
      out.push_back({name, false, "driver not initialized"});
    }
    return false;
  }

  auto resolved = ResolveSensorNames(names, &out);
  if (resolved.empty() && out.empty()) {
    // names was empty and nothing is configured
    return false;
  }

  // Record intent before sending: a sensor that (re)connects from now on is
  // put into `mode` by the discovery callback (see LidarInfoChangeCallback).
  {
    std::lock_guard<std::mutex> lock(desired_mode_mutex_);
    for (const auto& entry : resolved) {
      desired_mode_[entry.second] = mode;
    }
  }

  // Send to connected sensors; sensors still discovering get the command as
  // soon as connect_state leaves kConnectStateOff (polled until the deadline,
  // which absorbs the configure->activate race right after SDK init).
  auto* state = new WorkModeAckState();
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::unordered_map<uint32_t, bool> sent;  // handle -> command dispatched

  auto connect_state_of = [this](uint32_t handle) -> LidarConnectState {
    for (uint8_t i = 0; i < kMaxSourceLidar; ++i) {
      if (lidars_[i].handle == handle &&
          lidars_[i].lidar_type == kLivoxLidarType) {
        return lidars_[i].connect_state;
      }
    }
    return kConnectStateOff;
  };

  while (true) {
    for (const auto& entry : resolved) {
      const uint32_t handle = entry.second;
      if (!sent[handle] && connect_state_of(handle) != kConnectStateOff) {
        std::cout << "sending work mode " << static_cast<int>(mode)
                  << " to sensor '" << entry.first << "' (handle: " << handle
                  << ")" << std::endl;
        sent[handle] = true;
        {
          std::lock_guard<std::mutex> lock(state->mu);
          ++state->outstanding;
        }
        SetLivoxLidarWorkMode(handle, mode, WorkModeAckCallback, state);
      }
    }

    bool all_acked;
    {
      std::unique_lock<std::mutex> lock(state->mu);
      state->cv.wait_until(lock, std::min(deadline,
                                          std::chrono::steady_clock::now() +
                                              std::chrono::milliseconds(50)));
      all_acked = true;
      for (const auto& entry : resolved) {
        if (state->done.find(entry.second) == state->done.end()) {
          all_acked = false;
          break;
        }
      }
    }
    if (all_acked || std::chrono::steady_clock::now() >= deadline) {
      break;
    }
  }

  bool all_ok = out.empty();  // unknown-name failures already recorded
  bool destroy;
  {
    std::lock_guard<std::mutex> lock(state->mu);
    for (const auto& entry : resolved) {
      SensorModeResult r{entry.first, false, ""};
      auto it = state->done.find(entry.second);
      if (it == state->done.end()) {
        r.detail = sent[entry.second] ? "ack timeout" : "not connected";
      } else if (it->second != kLivoxLidarStatusSuccess) {
        r.detail = "command failed";
      } else {
        r.ok = true;
      }
      if (!r.ok) {
        all_ok = false;
        std::cout << "work-mode result for '" << r.name << "': " << r.detail
                  << std::endl;
      }
      out.push_back(std::move(r));
    }
    state->abandoned = true;
    destroy = state->outstanding == 0;
  }
  if (destroy) {
    delete state;
  }
  return all_ok;
}

void LdsLidar::SleepAllLidarsBlocking(std::chrono::milliseconds timeout) {
  // kLivoxLidarWakeUp is the MID-360 standby (motor off) mode; see header.
  SetLidarsWorkModeBlocking({}, kLivoxLidarWakeUp, timeout);
}

bool LdsLidar::WakeAllLidarsBlocking(std::chrono::milliseconds timeout) {
  return SetLidarsWorkModeBlocking({}, kLivoxLidarNormal, timeout);
}

}  // namespace livox_ros

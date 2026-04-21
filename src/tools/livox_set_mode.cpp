/**
 * @file livox_set_mode.cpp
 * @brief CLI tool to change the work mode of a Livox LiDAR via the Livox SDK2.
 *
 * Ported from the standalone livox_power_tool package so that all LiDAR
 * control tooling lives inside the driver package. Built as a separate
 * executable so it can be used with or without the driver node running:
 *
 *   ros2 run livox_ros_driver2 livox_set_mode <config.json> <sleep|normal>
 *
 * Modes:
 *   - "sleep"  : Puts the LiDAR into low-power standby (motor stops).
 *                Maps to kLivoxLidarWakeUp (0x02) in the SDK — despite the
 *                SDK name, this is the standby mode on the MID-360.
 *   - "normal" : Restores normal scanning operation.
 *                Maps to kLivoxLidarNormal (0x01) in the SDK.
 *
 * On the MID-360, kLivoxLidarSleep (0x03) does NOT work; kLivoxLidarWakeUp
 * is the correct enum for the motor-off state. See Livox-SDK2 issue #103.
 */

#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

static std::atomic<bool> g_done{false};
static LivoxLidarWorkMode g_target_mode = kLivoxLidarWakeUp;
static const char *g_target_mode_label = "Sleep";

static void WorkModeCallback(livox_status status, uint32_t handle,
                             LivoxLidarAsyncControlResponse *response,
                             void * /*client_data*/) {
  if (status == kLivoxLidarStatusSuccess) {
    if (response) {
      printf("[livox_set_mode] Work mode command acknowledged "
             "(handle: %u, ret_code: %u, error_key: %u)\n",
             handle, response->ret_code, response->error_key);
    } else {
      printf("[livox_set_mode] Work mode command acknowledged "
             "(handle: %u, no response data)\n", handle);
    }
  } else {
    fprintf(stderr, "[livox_set_mode] Failed to change work mode "
            "(handle: %u, status: %d)\n", handle, status);
  }
  g_done = true;
}

static void LidarInfoChangeCallback(const uint32_t handle,
                                    const LivoxLidarInfo *info,
                                    void * /*client_data*/) {
  if (info == nullptr) return;

  printf("[livox_set_mode] Found LiDAR -- SN: %s, IP: %s, handle: %u\n",
         info->sn, info->lidar_ip, handle);
  printf("[livox_set_mode] Setting work mode to %s...\n", g_target_mode_label);

  SetLivoxLidarWorkMode(handle, g_target_mode, WorkModeCallback, nullptr);
}

static void PrintUsage(const char *prog) {
  printf("Usage: %s <config_json_path> <sleep|normal>\n\n", prog);
  printf("Modes:\n");
  printf("  sleep   Put the LiDAR into low-power standby (motor stops)\n");
  printf("  normal  Restore normal scanning operation\n");
}

int main(int argc, char *argv[]) {
  if (argc != 3) {
    PrintUsage(argv[0]);
    return 1;
  }

  const std::string config_path = argv[1];
  const std::string mode_str = argv[2];

  if (mode_str == "sleep") {
    g_target_mode = kLivoxLidarWakeUp;
    g_target_mode_label = "Sleep (SDK: kLivoxLidarWakeUp)";
  } else if (mode_str == "normal") {
    g_target_mode = kLivoxLidarNormal;
    g_target_mode_label = "Normal (SDK: kLivoxLidarNormal)";
  } else {
    fprintf(stderr, "Error: mode must be 'sleep' or 'normal'\n\n");
    PrintUsage(argv[0]);
    return 1;
  }

  printf("[livox_set_mode] Initializing SDK with config: %s\n", config_path.c_str());

  if (!LivoxLidarSdkInit(config_path.c_str())) {
    fprintf(stderr, "[livox_set_mode] Failed to initialize Livox SDK\n");
    return 1;
  }

  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  if (!LivoxLidarSdkStart()) {
    fprintf(stderr, "[livox_set_mode] Failed to start Livox SDK\n");
    LivoxLidarSdkUninit();
    return 1;
  }

  auto start = std::chrono::steady_clock::now();
  while (!g_done) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed > std::chrono::seconds(10)) {
      fprintf(stderr, "[livox_set_mode] Timeout -- no LiDAR responded within 10s\n");
      break;
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  LivoxLidarSdkUninit();
  return g_done ? 0 : 1;
}

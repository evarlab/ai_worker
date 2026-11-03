// Copyright 2026 evarlab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ffw_sharpa_hardware/sharpa_wave_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <utility>

#include "SharpaWaveSDK.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ffw_sharpa_hardware
{

namespace
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using Clock = std::chrono::steady_clock;

// SDK joint order (indices 0..21); URDF joints are "<side>_<name>", optionally prefixed.
constexpr std::array<const char *, kHandDof> kSdkJointNames = {
  "thumb_CMC_FE", "thumb_CMC_AA", "thumb_MCP_FE", "thumb_MCP_AA", "thumb_IP",
  "index_MCP_FE", "index_MCP_AA", "index_PIP", "index_DIP",
  "middle_MCP_FE", "middle_MCP_AA", "middle_PIP", "middle_DIP",
  "ring_MCP_FE", "ring_MCP_AA", "ring_PIP", "ring_DIP",
  "pinky_CMC", "pinky_MCP_FE", "pinky_MCP_AA", "pinky_PIP", "pinky_DIP",
};

// Device status codes seen in normal operation (0x0030 is reported by healthy hands).
constexpr std::array<int, 2> kOkDeviceCodes = {0x0000, 0x0030};

// Names of sharpa::FaultCode (SharpaWaveSDK.h, SDK 5.0.11).
const std::unordered_map<int, const char *> & fault_names()
{
  static const std::unordered_map<int, const char *> names = {
    {0, "NORMAL"},
    {255, "UNKNOWN_ERROR"},
    {0x0101, "ERROR_SYSTEM_LOAD_HIGH"},
    {0x0102, "ERROR_SYSTEM_USERDATA_SPACE_LOW"},
    {0x0103, "ERROR_SYSTEM_USERDATA_INODE_LOW"},
    {0x0141, "ERROR_SYSTEM_USERDATA_SPACE_CRIT"},
    {0x0142, "ERROR_SYSTEM_USERDATA_INODE_CRIT"},
    {0x0181, "ERROR_SYSTEM_ZERO_ERROR"},
    {0x0182, "ERROR_SYSTEM_CALIB_PARAM_LOAD_FAILED"},
    {0x0183, "ERROR_SYSTEM_MOTION_CTRL_THREAD_HANG"},
    {0x0201, "ERROR_POWER_MCU_VERSION_LOW"},
    {0x0202, "ERROR_POWER_MCU_COMMUNICATION_ERROR"},
    {0x0203, "ERROR_POWER_LOW_VOLTAGE"},
    {0x0204, "ERROR_POWER_LOW_VOLTAGE_FLAPPING"},
    {0x0205, "ERROR_POWER_HW_SHORT_PROTECTION"},
    {0x0240, "ERROR_POWER_UNDERVOLTAGE_THRESHOLD"},
    {0x0241, "ERROR_POWER_HAND_OVERCURRENT"},
    {0x0242, "ERROR_POWER_HW_SHORT_PROTECTION_REPEATED"},
    {0x0243, "ERROR_POWER_MOTOR_POWER_OFF_FAILED"},
    {0x0280, "ERROR_POWER_OVER_CURRENT"},
    {0x0281, "ERROR_POWER_OVER_VOLTAGE"},
    {0x0301, "ERROR_MOTOR_GEAR_RATIO_WARNING"},
    {0x0302, "ERROR_MOTOR_ANGLE_DEVIATION_WARNING"},
    {0x0340, "ERROR_MOTOR_SW"},
    {0x0341, "ERROR_MOTOR_OVERVOLTAGE"},
    {0x0342, "ERROR_MOTOR_UNDERVOLTAGE"},
    {0x0343, "ERROR_MOTOR_OVERHEAT"},
    {0x0344, "ERROR_MOTOR_START_FAIL"},
    {0x0345, "ERROR_MOTOR_SPEED_FB"},
    {0x0348, "ERROR_MOTOR_ENC_HS"},
    {0x0349, "ERROR_MOTOR_ENC_LS"},
    {0x034C, "ERROR_MOTOR_ALIGN"},
    {0x034F, "ERROR_MOTOR_UNKNOWN"},
    {0x0351, "ERROR_MOTOR_COMMUNICATION_ERROR"},
    {0x0352, "ERROR_MOTOR_GEAR_RATIO_ERROR"},
    {0x0353, "ERROR_MOTOR_ANGLE_DEVIATION_ERROR"},
    {0x0401, "ERROR_STRAIN_PHYSICAL_DAMAGE_SUSPECT"},
    {0x0402, "ERROR_STRAIN_FORCE_OUT_OF_RANGE"},
    {0x0403, "ERROR_STRAIN_WIRE_BREAK"},
    {0x0440, "ERROR_STRAIN_CALIB_PARAM_ERROR"},
    {0x0441, "ERROR_STRAIN_SAMPLING_ABNORMAL"},
    {0x0501, "ERROR_JOINT_LIMIT_EXCEEDED"},
    {0x0502, "ERROR_VELOCITY_LIMIT_EXCEEDED"},
    {0x0503, "ERROR_TORQUE_LIMIT_EXCEEDED"},
    {0x0504, "ERROR_CONTROL_JITTER_OVER_LIMIT"},
    {0x0505, "ERROR_CONTROL_TASK_TIMEOUT"},
    {0x0601, "ERROR_TEMP_WARM"},
    {0x0602, "ERROR_TEMP_HOT"},
    {0x0603, "ERROR_TEMP_VERY_HOT"},
    {0x0640, "ERROR_TEMP_COOLING_FAILURE"},
    {0x0701, "ERROR_TOUCH_CONFIG_READ_ERROR"},
    {0x0702, "ERROR_TOUCH_CONFIG_WRITE_ERROR"},
    {0x0703, "ERROR_TOUCH_FLASH_READ_ERROR"},
    {0x0704, "ERROR_TOUCH_FLASH_WRITE_ERROR"},
    {0x0705, "ERROR_TOUCH_CONFIG_UNPACK_ERROR"},
    {0x0706, "ERROR_TOUCH_CONFIG_INVALID"},
    {0x0707, "WARNING_TOUCH_SENSOR_CONNECTION_UNSTABLE"},
    {0x0708, "ERROR_TOUCH_FPS_LOW"},
    {0x0709, "ERROR_TOUCH_UDP_SEND_STALL"},
    {0x070A, "ERROR_TOUCH_G0_I6_CFG_READ"},
    {0x070B, "ERROR_TOUCH_ELASTOMER_PARTS_MATCH_FAILED"},
    {0x0740, "ERROR_TOUCH_SENSOR_NOT_DETECTED"},
    {0x0741, "ERROR_TOUCH_SENSOR_MCU1_COMMUNICATION_ERROR"},
    {0x0742, "ERROR_TOUCH_NPU_MODEL_FILE_MISSING"},
    {0x0743, "ERROR_TOUCH_SENSOR_I2C_COMMUNICATION_ERROR"},
    {0x0744, "ERROR_TOUCH_V4L2_BUF_STALL"},
    {0x0745, "ERROR_TOUCH_RAW_ENCODER_STALL"},
    {0x0746, "ERROR_TOUCH_DEFORM_ENCODER_STALL"},
    {0x0747, "ERROR_TOUCH_HW_VERIFY_FAILED"},
    {0x0748, "ERROR_TOUCH_ELASTOMER_PARTS_CFG_INVALID"},
    {0x0901, "ERROR_NETWORK_LINK_DOWN"},
    {0x0902, "ERROR_NETWORK_SPEED_ABNORMAL"},
    {0x0903, "ERROR_NETWORK_DUPLEX_HALF"},
    {0x0904, "ERROR_NETWORK_AUTONEG_ABNORMAL"},
    {0x0905, "ERROR_NETWORK_RX_ERR_BURST"},
    {0x0906, "ERROR_NETWORK_TX_ERR_BURST"},
    {0x0907, "ERROR_NETWORK_LINK_DOWN_LATCHED"},
    {0x0940, "ERROR_NETWORK_PHY_ERROR"},
    {0x0941, "ERROR_NETWORK_ADMIN_DOWN"},
    {0x0942, "ERROR_NETWORK_UNEXPECTED_PHY"},
    {0x0800, "SW_CONFIGURED_ERROR"},
    {0x0801, "SW_DEVICE_CONNECT_ERROR"},
    {0x0802, "SW_RUNNING_ERROR"},
    {0x0803, "SW_STOP_ERROR"},
    {0x0811, "SW_HAND_CONSTRUCTION_ERROR"},
    {0x0812, "SW_HAND_INITIALIZE_ERROR"},
    {0x0815, "SW_TOUCH_CONSTRUCTION_ERROR"},
    {0x0816, "SW_TOUCH_INITIALIZE_ERROR"},
    {0x0817, "SW_TOUCH_HOST_IP_INVALID"},
    {0x0818, "SW_TOUCH_HOST_PORT_OCCUPIED"},
    {0x0819, "SW_TOUCH_HOST_LISTEN_FAULT"},
    {0x0821, "SW_CONTROL_MODE_NOT_SUPPORTED"},
    {0x0822, "SW_SEND_HEARTBEAT_PACKET_FAILED"},
    {0x0830, "SW_HARDWARE_VERSION_MISMATCH"},
    {0x0831, "SW_FIRMWARE_VERSION_MISMATCH"},
    {0x0832, "SW_FIRMWARE_VERSION_CONFIG_NOT_FOUND"},
  };
  return names;
}

std::string hex(int code)
{
  char buffer[16];
  std::snprintf(buffer, sizeof(buffer), "0x%04x", code);
  return buffer;
}

bool ends_with(const std::string & text, const std::string & suffix)
{
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}

bool check(const sharpa::Error & error, const std::string & what, const rclcpp::Logger & logger)
{
  if (error.code == 0) {
    return true;
  }
  RCLCPP_ERROR(logger, "%s: SDK error %d: %s", what.c_str(), error.code, error.message.c_str());
  return false;
}

}  // namespace

SharpaWaveSystem::SharpaWaveSystem() = default;

SharpaWaveSystem::~SharpaWaveSystem()
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (hand_ != nullptr) {
    disable_motors();
    disconnect();
  }
}

CallbackReturn SharpaWaveSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (!parse_parameters() || !map_joints()) {
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    get_logger(), "%s hand: %s %s, speed_coeff %.2f", side_.c_str(),
    serial_number_.empty() ? "IP" : "serial", serial_number_.empty() ? ip_.c_str() :
    serial_number_.c_str(), speed_coeff_);
  return CallbackReturn::SUCCESS;
}

bool SharpaWaveSystem::parse_parameters()
{
  side_ = parameter(info_, "hand_side", "");
  if (side_ != "left" && side_ != "right") {
    RCLCPP_ERROR(get_logger(), "Parameter hand_side must be 'left' or 'right'");
    return false;
  }
  ip_ = parameter(info_, "ip", side_ == "right" ? "192.168.10.20" : "192.168.10.10");
  serial_number_ = parameter(info_, "serial_number", "");
  try {
    speed_coeff_ = std::stod(parameter(info_, "speed_coeff", "0.3"));
    current_coeff_ = std::stod(parameter(info_, "current_coeff", "0"));
    connect_timeout_ = std::stod(parameter(info_, "connect_timeout", "10.0"));
    feedback_timeout_ = std::stod(parameter(info_, "feedback_timeout", "0.5"));
    diagnostics_period_ = std::stod(parameter(info_, "diagnostics_period", "1.0"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Invalid numeric hardware parameter: %s", e.what());
    return false;
  }
  // The SDK clamps SE hands to speed <= 0.5 and current <= 0.6; reject rather than clamp.
  if (!(speed_coeff_ > 0.0 && speed_coeff_ <= 0.5)) {
    RCLCPP_ERROR(get_logger(), "speed_coeff must be in (0, 0.5]");
    return false;
  }
  if (!(current_coeff_ >= 0.0 && current_coeff_ <= 0.6)) {
    RCLCPP_ERROR(get_logger(), "current_coeff must be in [0, 0.6] (0 keeps the device setting)");
    return false;
  }
  if (connect_timeout_ <= 0.0 || feedback_timeout_ <= 0.0 || diagnostics_period_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "Timeouts and the diagnostics period must be positive");
    return false;
  }
  return true;
}

bool SharpaWaveSystem::map_joints()
{
  if (info_.joints.size() != kHandDof) {
    RCLCPP_ERROR(
      get_logger(), "Expected %zu joints for the %s hand, got %zu", kHandDof, side_.c_str(),
      info_.joints.size());
    return false;
  }
  std::array<bool, kHandDof> seen{};
  joints_.clear();
  for (const auto & joint : info_.joints) {
    Joint mapped;
    bool found = false;
    for (std::size_t k = 0; k < kHandDof; ++k) {
      if (ends_with(joint.name, side_ + "_" + kSdkJointNames[k])) {
        mapped.sdk_index = k;
        found = true;
        break;
      }
    }
    if (!found || seen[mapped.sdk_index]) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' is not a distinct %s hand joint", joint.name.c_str(),
        side_.c_str());
      return false;
    }
    seen[mapped.sdk_index] = true;
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(get_logger(), "Joint '%s' needs exactly one position command interface",
        joint.name.c_str());
      return false;
    }
    mapped.position_command = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    for (const auto & state : joint.state_interfaces) {
      const std::string name = joint.name + "/" + state.name;
      if (state.name == hardware_interface::HW_IF_POSITION) {
        mapped.position_state = name;
      } else if (state.name == hardware_interface::HW_IF_VELOCITY) {
        mapped.velocity_state = name;
      } else if (state.name == hardware_interface::HW_IF_EFFORT) {
        mapped.effort_state = name;
      } else {
        RCLCPP_ERROR(get_logger(), "Joint '%s': unsupported state interface '%s'",
          joint.name.c_str(), state.name.c_str());
        return false;
      }
    }
    if (mapped.position_state.empty()) {
      RCLCPP_ERROR(get_logger(), "Joint '%s' needs a position state interface",
        joint.name.c_str());
      return false;
    }
    const auto limits = info_.limits.find(joint.name);
    if (limits != info_.limits.end() && limits->second.has_position_limits) {
      mapped.lower = limits->second.min_position;
      mapped.upper = limits->second.max_position;
    }
    joints_.push_back(mapped);
  }
  return true;
}

CallbackReturn SharpaWaveSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (get_node() && !diagnostics_pub_) {
    diagnostics_pub_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::SystemDefaultsQoS());
  }
  if (hand_ == nullptr && !connect()) {
    disconnect();
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool SharpaWaveSystem::connect()
{
  auto & manager = sharpa::SharpaWaveManager::get_instance();
  const auto want = side_ == "left" ? sharpa::HandSide::LEFT : sharpa::HandSide::RIGHT;
  const auto deadline = Clock::now() + std::chrono::duration<double>(connect_timeout_);
  sharpa::DeviceInfo device;
  std::string seen;
  while (true) {
    std::vector<sharpa::DeviceInfo> matches;
    std::ostringstream listing;
    for (const auto & info : manager.get_all_devices()) {
      listing << " " << info.sn << "(" << (info.hand_side == sharpa::HandSide::LEFT ? "L" : "R")
              << "," << info.ip << ")";
      if (info.device_type != sharpa::DeviceType::HAND || info.hand_side != want) {
        continue;
      }
      if (serial_number_.empty() ? info.ip == ip_ : info.sn == serial_number_) {
        matches.push_back(info);
      }
    }
    seen = listing.str();
    if (matches.size() == 1) {
      device = matches.front();
      break;
    }
    if (Clock::now() >= deadline) {
      RCLCPP_ERROR(
        get_logger(),
        "Expected one %s hand at %s; found %zu. Devices heard:%s. Check the hand's power, "
        "cable and the host address 192.168.10.240/24 (ffw_sharpa_hardware/scripts/"
        "hand_network.sh).", side_.c_str(),
        serial_number_.empty() ? ip_.c_str() : serial_number_.c_str(), matches.size(),
        seen.empty() ? " none" : seen.c_str());
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  try {
    sharpa::SharpaWaveConfig config;
    config.disable_tactile = true;
    hand_ = &manager.connect(device.sn, config);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Connecting to %s failed: %s", device.sn.c_str(), e.what());
    hand_ = nullptr;
    return false;
  }
  connected_sn_ = device.sn;
  firmware_ = device.firmware_version;
  // Start disabled, as after power-up; activation enables the motors.
  if (!check(hand_->set_enable_state(false), "Disable at connect", get_logger())) {
    return false;
  }
  if (!hand_->start()) {
    RCLCPP_ERROR(get_logger(), "Could not start feedback from %s", device.sn.c_str());
    return false;
  }
  const auto feedback_deadline = Clock::now() + std::chrono::seconds(2);
  while (hand_->get_states().angles.size() != kHandDof) {
    if (Clock::now() >= feedback_deadline) {
      RCLCPP_ERROR(get_logger(), "No joint feedback from %s", device.sn.c_str());
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  have_sequence_ = false;
  stale_ = true;
  enabled_ = false;
  rate_window_start_ = Clock::now();
  feedback_count_ = 0;
  RCLCPP_INFO(
    get_logger(), "Connected %s hand %s (%s, firmware %s); motors disabled", side_.c_str(),
    device.sn.c_str(), device.ip.c_str(), firmware_.c_str());
  return true;
}

void SharpaWaveSystem::disconnect()
{
  if (hand_ != nullptr) {
    try {
      hand_->stop();
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Stopping feedback failed: %s", e.what());
    }
  }
  if (!connected_sn_.empty()) {
    try {
      sharpa::SharpaWaveManager::get_instance().disconnect(connected_sn_);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Disconnect failed: %s", e.what());
    }
  }
  hand_ = nullptr;
  connected_sn_.clear();
}

bool SharpaWaveSystem::disable_motors()
{
  active_ = false;
  if (hand_ == nullptr) {
    return true;
  }
  if (!check(hand_->set_enable_state(false), "Disable motors", get_logger())) {
    return false;
  }
  // The new state takes about 70 ms to apply.
  for (int i = 0; i < 20; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const auto [error, enabled] = hand_->get_enable_state();
    if (error.code == 0 && !enabled) {
      enabled_ = false;
      return true;
    }
  }
  RCLCPP_ERROR(get_logger(), "%s hand still reports enabled motors", side_.c_str());
  return false;
}

std::string SharpaWaveSystem::device_fault() const
{
  const sharpa::DeviceStatus status = hand_->get_device_info().status;
  const int code = status.error_code;
  if (std::find(kOkDeviceCodes.begin(), kOkDeviceCodes.end(), code) != kOkDeviceCodes.end()) {
    return "";
  }
  const auto name = fault_names().find(code);
  std::string text = name == fault_names().end() ? "fault" : name->second;
  text += " (" + hex(code) + ")";
  std::string joints;
  for (std::size_t k = 0; k < kHandDof; ++k) {
    if (status.error_joint >> k & 1u) {
      joints += (joints.empty() ? "" : ", ") + side_ + "_" + kSdkJointNames[k];
    }
  }
  return joints.empty() ? text : text + " on " + joints;
}

CallbackReturn SharpaWaveSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (hand_ == nullptr) {
    return CallbackReturn::ERROR;
  }
  const std::string fault = device_fault();
  if (!fault.empty()) {
    RCLCPP_ERROR(
      get_logger(), "Not enabling the %s hand: %s. Clear it (e.g. in Sharpa Pilot) first.",
      side_.c_str(), fault.c_str());
    return CallbackReturn::ERROR;
  }
  const sharpa::State state = hand_->get_states();
  if (state.angles.size() != kHandDof) {
    RCLCPP_ERROR(get_logger(), "No joint feedback; not enabling the %s hand", side_.c_str());
    return CallbackReturn::ERROR;
  }
  const std::vector<float> hold(state.angles.begin(), state.angles.end());
  const auto & logger = get_logger();
  bool ok = check(hand_->set_control_source(sharpa::ControlSource::SDK), "SDK control", logger) &&
    check(hand_->set_speed_coeff(static_cast<float>(speed_coeff_)), "Speed coefficient", logger);
  if (ok && current_coeff_ > 0.0) {
    ok = check(
      hand_->set_current_coeff(static_cast<float>(current_coeff_)), "Current coefficient",
      logger);
  }
  // set_control_mode() enables the motors; hold the measured pose right away.
  ok = ok &&
    check(hand_->set_control_mode(sharpa::ControlMode::POSITION), "Position mode", logger) &&
    check(hand_->set_joint_position(hold, true), "Hold the measured pose", logger);
  bool enabled = false;
  for (int i = 0; ok && !enabled && i < 20; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const auto [error, state_enabled] = hand_->get_enable_state();
    enabled = error.code == 0 && state_enabled;
  }
  if (!ok || !enabled) {
    RCLCPP_ERROR(get_logger(), "Enabling the %s hand failed; disabling it", side_.c_str());
    disable_motors();
    return CallbackReturn::ERROR;
  }
  for (const auto & joint : joints_) {
    command_[joint.sdk_index] = hold[joint.sdk_index];
    set_command(joint.position_command, static_cast<double>(hold[joint.sdk_index]));
  }
  enabled_ = true;
  hold_reason_.clear();
  active_ = true;
  next_enable_check_ = Clock::now() + std::chrono::seconds(1);
  RCLCPP_INFO(get_logger(), "%s hand enabled in position mode", side_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SharpaWaveSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  const bool ok = disable_motors();
  RCLCPP_INFO(get_logger(), "%s hand disabled", side_.c_str());
  return ok ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn SharpaWaveSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  disable_motors();
  disconnect();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SharpaWaveSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  disable_motors();
  disconnect();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SharpaWaveSystem::on_error(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  disable_motors();
  return CallbackReturn::SUCCESS;
}

return_type SharpaWaveSystem::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (hand_ == nullptr) {
    return return_type::OK;
  }
  const auto now = Clock::now();
  const sharpa::State state = hand_->get_states();
  const bool complete = state.angles.size() == kHandDof;
  if (complete && (!have_sequence_ || state.sequence != last_sequence_)) {
    // The device numbers its feedback packets (uint16); count them to report the device rate.
    if (have_sequence_) {
      feedback_count_ += static_cast<uint16_t>(state.sequence - last_sequence_);
    }
    last_sequence_ = state.sequence;
    have_sequence_ = true;
    last_feedback_ = now;
  }
  stale_ = !have_sequence_ ||
    now - last_feedback_ > std::chrono::duration<double>(feedback_timeout_);
  const bool velocities = state.velocities.size() == kHandDof;
  const bool torques = state.torques.size() == kHandDof;
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto & joint : joints_) {
    const std::size_t k = joint.sdk_index;
    set_state(joint.position_state, stale_ ? nan : static_cast<double>(state.angles[k]));
    if (!joint.velocity_state.empty()) {
      set_state(joint.velocity_state,
        stale_ || !velocities ? nan : static_cast<double>(state.velocities[k]));
    }
    if (!joint.effort_state.empty()) {
      set_state(joint.effort_state,
        stale_ || !torques ? nan : static_cast<double>(state.torques[k]));
    }
  }

  fault_ = device_fault();
  if (now >= next_enable_check_) {
    const auto [error, enabled] = hand_->get_enable_state();
    if (error.code == 0) {
      enabled_ = enabled;
    }
    next_enable_check_ = now + std::chrono::seconds(1);
  }
  if (active_ && hold_reason_.empty()) {
    if (!fault_.empty()) {
      hold_reason_ = fault_;
    } else if (stale_) {
      hold_reason_ = "joint feedback lost";
    } else if (!enabled_) {
      hold_reason_ = "motors were disabled outside ros2_control";
    }
    if (!hold_reason_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "%s hand: %s. Holding: commands are not sent until the hand is restarted "
        "(ffw_sharpa_hardware/scripts/restart_hand.sh %s).", side_.c_str(),
        hold_reason_.c_str(), side_.c_str());
    }
  }

  const double window = std::chrono::duration<double>(now - rate_window_start_).count();
  if (window >= 1.0) {
    feedback_rate_hz_ = static_cast<double>(feedback_count_) / window;
    feedback_count_ = 0;
    rate_window_start_ = now;
  }
  if (now >= next_diagnostics_) {
    publish_diagnostics(time);
    next_diagnostics_ = now + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(diagnostics_period_));
  }
  return return_type::OK;
}

return_type SharpaWaveSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (hand_ == nullptr || !active_ || !hold_reason_.empty()) {
    return return_type::OK;
  }
  std::array<float, kHandDof> target = command_;
  for (const auto & joint : joints_) {
    const double value = get_command<double>(joint.position_command);
    if (!std::isfinite(value)) {
      return return_type::OK;  // no controller has written a target yet
    }
    target[joint.sdk_index] = static_cast<float>(std::clamp(value, joint.lower, joint.upper));
  }
  const sharpa::Error error =
    hand_->set_joint_position(std::vector<float>(target.begin(), target.end()), false);
  if (error.code != 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "%s hand command failed: SDK error %d: %s",
      side_.c_str(), error.code, error.message.c_str());
    return return_type::OK;
  }
  command_ = target;
  return return_type::OK;
}

void SharpaWaveSystem::publish_diagnostics(const rclcpp::Time & time)
{
  if (!diagnostics_pub_) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "sharpa_wave: " + side_ + " hand";
  status.hardware_id = connected_sn_;
  if (!fault_.empty()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = fault_;
  } else if (stale_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "no joint feedback";
  } else if (!hold_reason_.empty()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "holding: " + hold_reason_;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = active_ ? "enabled" : "connected, motors disabled";
  }
  const auto value = [](const std::string & key, const std::string & text) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = text;
      return kv;
    };
  const sharpa::DeviceInfo info = hand_->get_device_info();
  char rate[32];
  std::snprintf(rate, sizeof(rate), "%.0f", feedback_rate_hz_);
  char speed[32];
  std::snprintf(speed, sizeof(speed), "%.2f", speed_coeff_);
  status.values = {
    value("ip", info.ip),
    value("firmware", firmware_),
    value("motors_enabled", enabled_ ? "true" : "false"),
    value("ros2_control_active", active_ ? "true" : "false"),
    value("device_status", hex(info.status.error_code)),
    value("device_feedback_hz", rate),
    value("speed_coeff", speed),
  };
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = time;
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

}  // namespace ffw_sharpa_hardware

PLUGINLIB_EXPORT_CLASS(ffw_sharpa_hardware::SharpaWaveSystem, hardware_interface::SystemInterface)

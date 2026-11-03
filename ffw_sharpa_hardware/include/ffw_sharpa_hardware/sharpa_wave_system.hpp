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

#ifndef FFW_SHARPA_HARDWARE__SHARPA_WAVE_SYSTEM_HPP_
#define FFW_SHARPA_HARDWARE__SHARPA_WAVE_SYSTEM_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace sharpa
{
class SharpaWave;
}  // namespace sharpa

namespace ffw_sharpa_hardware
{

constexpr std::size_t kHandDof = 22;

/// One Sharpa Wave hand as a ros2_control system, on the vendor's C++ SDK.
///
/// Lifecycle:
///   configure  - find the hand (side + IP or serial number), connect, keep it disabled,
///                start feedback.
///   activate   - refuse on a device fault; SDK control source, speed coefficient, position
///                mode (enables the motors) holding the measured pose.
///   deactivate - disable the motors (fingers go limp; a held object drops).
///   cleanup    - stop feedback and disconnect.
///
/// A device fault or lost feedback never returns ERROR from read()/write(): that would take
/// the joint_state_broadcaster down with it. Instead commands are held (not sent) until the
/// hand is restarted (scripts/restart_hand.sh: controller off, component inactive -> active,
/// controller on), positions read NaN while feedback is stale, and the state is published on
/// /diagnostics.
class SharpaWaveSystem : public hardware_interface::SystemInterface
{
public:
  SharpaWaveSystem();
  ~SharpaWaveSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::size_t sdk_index = 0;
    std::string position_state;
    std::string velocity_state;  // empty when not exported
    std::string effort_state;    // empty when not exported
    std::string position_command;
    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();
  };

  bool parse_parameters();
  bool map_joints();
  bool connect();
  void disconnect();
  bool disable_motors();
  /// Device fault text, or empty when the device reports normal operation.
  std::string device_fault() const;
  void publish_diagnostics(const rclcpp::Time & time);

  // Parameters
  std::string side_;
  std::string ip_;
  std::string serial_number_;
  double speed_coeff_ = 0.3;
  double current_coeff_ = 0.0;  // 0: leave the device setting
  double connect_timeout_ = 10.0;
  double feedback_timeout_ = 0.5;
  double diagnostics_period_ = 1.0;

  std::vector<Joint> joints_;

  // Device
  mutable std::mutex sdk_mutex_;
  sharpa::SharpaWave * hand_ = nullptr;
  std::string connected_sn_;
  std::string firmware_;

  // Runtime state (read/write thread)
  bool active_ = false;
  std::string hold_reason_;  // non-empty: commands are not sent
  std::string fault_;
  bool stale_ = true;
  bool enabled_ = false;
  uint16_t last_sequence_ = 0;
  bool have_sequence_ = false;
  std::chrono::steady_clock::time_point last_feedback_;
  std::array<float, kHandDof> command_{};
  std::size_t feedback_count_ = 0;
  std::chrono::steady_clock::time_point rate_window_start_;
  double feedback_rate_hz_ = 0.0;
  std::chrono::steady_clock::time_point next_enable_check_;
  std::chrono::steady_clock::time_point next_diagnostics_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace ffw_sharpa_hardware

#endif  // FFW_SHARPA_HARDWARE__SHARPA_WAVE_SYSTEM_HPP_

// Copyright 2026
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

#ifndef MIVIA_ROVER_PLATFORM__MIVIA_ROVER_SYSTEM_HPP_
#define MIVIA_ROVER_PLATFORM__MIVIA_ROVER_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "realtime_tools/realtime_buffer.hpp"

#include "mivia_rover_can_msgs/msg/encoder_rpms.hpp"
#include "mivia_rover_can_msgs/msg/reference.hpp"

#include <condition_variable>
#include <mutex>
#include <fstream>
#include <chrono>

// extern "C" {
//   #include "rover.h"
// }s

namespace mivia_rover_platform
{

class MiviaRoverSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MiviaRoverSystem);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kNumWheelJoints = 4U;

  /* Message field order convention: [front_left, rear_left, front_right, rear_right] */
  struct EncoderSample
  {
    std::array<double, kNumWheelJoints> rpm;
    std::uint64_t stamp_ns;
    //Added for take the start time of the CAN message through DDS. It is just a dummy variable for testing and jitter calcultion. Also for time estimation of read function.
    std::uint64_t t_start_dds_ns;
    bool valid;
  };

  struct CommandSample
  {
    std::array<std::int32_t, kNumWheelJoints> rpm;
    std::uint64_t stamp_ns;
    bool valid;
  };

  struct LogEntryRead {
    uint64_t t_start_ns;
    uint64_t t_end_ns;
  };

  void encoder_callback_(const mivia_rover_can_msgs::msg::EncoderRpms::SharedPtr msg);

  void start_comm_thread_();
  void stop_comm_thread_();
  void comm_thread_entry_();

  bool build_joint_mapping_();

  static double rpm_to_rad_s_(double rpm);
  static double rad_s_to_rpm_(double rad_s);
  static std::int32_t clamp_rpm_(double rpm, double abs_limit);

  static bool try_get_param_string_(
    const hardware_interface::HardwareInfo & info,
    const std::string & key,
    std::string & out_value);

  static bool try_get_param_double_(
    const hardware_interface::HardwareInfo & info,
    const std::string & key,
    double & out_value);

  static bool try_get_param_u32_(
    const hardware_interface::HardwareInfo & info,
    const std::string & key,
    std::uint32_t & out_value);

  /* ---------- Parameters (from URDF/xacro hardware_parameters) ---------- */
  std::string encoder_topic_;
  std::string reference_topic_;
  double feedback_timeout_sec_;
  std::uint32_t max_consecutive_timeouts_;
  double publish_rate_hz_;
  double rpm_limit_abs_;

  std::array<std::string, kNumWheelJoints> wheel_joint_names_;   /* [FL, RL, FR, RR] */
  std::array<std::size_t, kNumWheelJoints> wheel_joint_indices_; /* indices into info_.joints */

  /* ---------- ROS comm (NON-RT) ---------- */
  rclcpp::Node::SharedPtr comm_node_;
  rclcpp::Subscription<mivia_rover_can_msgs::msg::EncoderRpms>::SharedPtr encoder_sub_;
  rclcpp::Publisher<mivia_rover_can_msgs::msg::Reference>::SharedPtr reference_pub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  std::thread comm_thread_;
  std::atomic<bool> comm_running_;

  //it will be used for made a passive waiting of the thread
  std::atomic<bool> dds_has_new_data_{false};

  /* ---------- RT buffers ---------- */
  realtime_tools::RealtimeBuffer<EncoderSample> encoder_buffer_;
  /* ---------- RT command double-buffer (lock-free) ---------- */
  std::array<CommandSample, 2U> cmd_buf_;
  std::atomic<std::uint32_t> cmd_seq_;
  std::atomic<bool> cmd_valid_;



  /* ---------- State/command storage exposed to ros2_control ---------- */
  std::vector<double> hw_commands_;    /* [rad/s] */
  std::vector<double> hw_positions_;   /* [rad] derived */
  std::vector<double> hw_velocities_;  /* [rad/s] measured */

  /* ---------- Fault handling ---------- */
  std::uint32_t consecutive_timeouts_;
  std::atomic<bool> fault_stop_;

  //Added for logging of times

  /* ---------- SocketCAN ---------- */
  int can_socket_fd_ = -1;

  /* ---------- Time Logging ---------- */
  static constexpr std::size_t kLogBufferSize = 4096U;
  std::array<LogEntryRead, kLogBufferSize> log_buffer_;
  std::atomic<std::size_t> log_head_{0};
  std::atomic<std::size_t> log_tail_{0};

  /* ---------- Logging Thread with sync without busywaiut---------- */
  std::thread logging_thread_;
  std::atomic<bool> logging_is_running_{false};
  std::mutex log_mutex_;
  std::condition_variable log_cv_;
  void logging_thread_entry_();
};

}  // namespace mivia_rover_platform

#endif  // MIVIA_ROVER_PLATFORM__MIVIA_ROVER_SYSTEM_HPP_
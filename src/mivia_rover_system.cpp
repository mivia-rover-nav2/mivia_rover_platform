// Copyright 2026
// Licensed under the Apache License, Version 2.0

#include "mivia_rover_platform/mivia_rover_system.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mivia_rover_platform
{

static constexpr double kTwoPi = 6.2831853071795864769;
static constexpr double kNsToSec = 1.0e-9;

static bool find_interface_name(
  const std::vector<hardware_interface::InterfaceInfo> & ifaces,
  const std::string & name)
{
  bool found = false;

  for (std::size_t i = 0U; (i < ifaces.size()) && (found == false); ++i)
  {
    if (ifaces[i].name == name)
    {
      found = true;
    }
  }

  return found;
}

static bool try_get_param_string(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  std::string & out_value)
{
  const std::unordered_map<std::string, std::string>::const_iterator it =
    info.hardware_parameters.find(key);

  if (it == info.hardware_parameters.end())
  {
    return false;
  }

  out_value = it->second;
  return true;
}

static bool try_get_param_double(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  double & out_value)
{
  std::string s;
  if (!try_get_param_string(info, key, s))
  {
    return false;
  }

  out_value = std::stod(s);
  return true;
}

static bool try_get_param_u32(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  std::uint32_t & out_value)
{
  std::string s;
  if (!try_get_param_string(info, key, s))
  {
    return false;
  }

  out_value = static_cast<std::uint32_t>(std::stoul(s));
  return true;
}

double MiviaRoverSystem::rpm_to_rad_s_(double rpm)
{
  return (rpm * kTwoPi) / 60.0;
}

double MiviaRoverSystem::rad_s_to_rpm_(double rad_s)
{
  return (rad_s * 60.0) / kTwoPi;
}

std::int32_t MiviaRoverSystem::clamp_rpm_(double rpm, double abs_limit)
{
  double x = rpm;

  if (x > abs_limit)
  {
    x = abs_limit;
  }
  else
  {
    if (x < (-abs_limit))
    {
      x = -abs_limit;
    }
  }

  /* Round-to-nearest */
  const double rounded = std::floor(x + ((x >= 0.0) ? 0.5 : -0.5));

  if (rounded > static_cast<double>(std::numeric_limits<std::int32_t>::max()))
  {
    return std::numeric_limits<std::int32_t>::max();
  }
  if (rounded < static_cast<double>(std::numeric_limits<std::int32_t>::min()))
  {
    return std::numeric_limits<std::int32_t>::min();
  }

  return static_cast<std::int32_t>(rounded);
}

bool MiviaRoverSystem::build_joint_mapping_()
{
  std::array<bool, kNumWheelJoints> found;
  found[0] = false;
  found[1] = false;
  found[2] = false;
  found[3] = false;

  for (std::size_t i = 0U; i < info_.joints.size(); ++i)
  {
    const std::string & jn = info_.joints[i].name;

    if (jn == wheel_joint_names_[0])
    {
      wheel_joint_indices_[0] = i;
      found[0] = true;
    }
    else
    {
      /* no-op */
    }

    if (jn == wheel_joint_names_[1])
    {
      wheel_joint_indices_[1] = i;
      found[1] = true;
    }
    else
    {
      /* no-op */
    }

    if (jn == wheel_joint_names_[2])
    {
      wheel_joint_indices_[2] = i;
      found[2] = true;
    }
    else
    {
      /* no-op */
    }

    if (jn == wheel_joint_names_[3])
    {
      wheel_joint_indices_[3] = i;
      found[3] = true;
    }
    else
    {
      /* no-op */
    }
  }

  for (std::size_t k = 0U; k < kNumWheelJoints; ++k)
  {
    if (!found[k])
    {
      return false;
    }
  }

  return true;
}

hardware_interface::CallbackReturn MiviaRoverSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  const hardware_interface::CallbackReturn base_ret =
    hardware_interface::SystemInterface::on_init(info);

  if (base_ret != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  /* ---------------- Parameters from URDF/xacro ---------------- */
  encoder_topic_ = "/mivia_rover/encoder_rpms";
  reference_topic_ = "/mivia_rover/reference";
  feedback_timeout_sec_ = 0.150;
  max_consecutive_timeouts_ = 10U;
  publish_rate_hz_ = 50.0;
  rpm_limit_abs_ = 180.0;

  (void)try_get_param_string(info_, "encoder_topic", encoder_topic_);
  (void)try_get_param_string(info_, "reference_topic", reference_topic_);

  {
    double timeout_ms = 150.0;
    if (try_get_param_double(info_, "feedback_timeout_ms", timeout_ms))
    {
      feedback_timeout_sec_ = timeout_ms / 1000.0;
    }
  }

  (void)try_get_param_u32(info_, "max_consecutive_timeouts", max_consecutive_timeouts_);
  (void)try_get_param_double(info_, "publish_rate_hz", publish_rate_hz_);
  (void)try_get_param_double(info_, "rpm_limit_abs", rpm_limit_abs_);

  wheel_joint_names_[0] = "";
  wheel_joint_names_[1] = "";
  wheel_joint_names_[2] = "";
  wheel_joint_names_[3] = "";

  (void)try_get_param_string(info_, "joint_front_left", wheel_joint_names_[0]);
  (void)try_get_param_string(info_, "joint_rear_left", wheel_joint_names_[1]);
  (void)try_get_param_string(info_, "joint_front_right", wheel_joint_names_[2]);
  (void)try_get_param_string(info_, "joint_rear_right", wheel_joint_names_[3]);

  /* ---------------- Validate joints and interfaces (robust) ---------------- */
  if (info_.joints.size() != kNumWheelJoints)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0U; i < info_.joints.size(); ++i)
  {
    const hardware_interface::ComponentInfo & joint = info_.joints[i];

    /* Command: exactly one 'velocity' */
    if (joint.command_interfaces.size() != 1U)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    /* State: must provide position + velocity, but do not assume order */
    if (joint.state_interfaces.size() < 2U)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!find_interface_name(joint.state_interfaces, hardware_interface::HW_IF_POSITION))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!find_interface_name(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (!build_joint_mapping_())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  /* ---------------- Pre-allocate vectors exposed to ros2_control ---------------- */
  hw_commands_.assign(kNumWheelJoints, 0.0);
  hw_positions_.assign(kNumWheelJoints, 0.0);
  hw_velocities_.assign(kNumWheelJoints, 0.0);

  /* ---------------- Init RT buffers and flags ---------------- */
  consecutive_timeouts_ = 0U;

  comm_running_.store(false);

  fault_stop_.store(false);

  {
    EncoderSample e;
    e.rpm[0] = 0.0;
    e.rpm[1] = 0.0;
    e.rpm[2] = 0.0;
    e.rpm[3] = 0.0;
    e.stamp_ns = 0ULL;
    e.valid = false;
    encoder_buffer_.writeFromNonRT(e);
  }

  /* Command double-buffer init */
  cmd_seq_.store(0U);
  cmd_valid_.store(false);

  {
    CommandSample c;
    c.rpm[0] = 0;
    c.rpm[1] = 0;
    c.rpm[2] = 0;
    c.rpm[3] = 0;
    c.stamp_ns = 0ULL;
    c.valid = false;

    cmd_buf_[0] = c;
    cmd_buf_[1] = c;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiviaRoverSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  /* Create comm node and executor (NON-RT) */
  comm_node_ = std::make_shared<rclcpp::Node>("mivia_rover_system_comm");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  /* QoS: encoder as sensor stream; reference as default system QoS */
  const rclcpp::QoS encoder_qos = rclcpp::SensorDataQoS();
  const rclcpp::QoS reference_qos = rclcpp::SystemDefaultsQoS();

  reference_pub_ =
    comm_node_->create_publisher<mivia_rover_can_msgs::msg::Reference>(reference_topic_, reference_qos);

  encoder_sub_ =
    comm_node_->create_subscription<mivia_rover_can_msgs::msg::EncoderRpms>(
      encoder_topic_,
      encoder_qos,
      std::bind(&MiviaRoverSystem::encoder_callback_, this, std::placeholders::_1));

  exec_->add_node(comm_node_);
  start_comm_thread_();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiviaRoverSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_comm_thread_();

  if ((exec_ != nullptr) && (comm_node_ != nullptr))
  {
    exec_->remove_node(comm_node_);
  }

  encoder_sub_.reset();
  reference_pub_.reset();
  exec_.reset();
  comm_node_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiviaRoverSystem::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::CallbackReturn MiviaRoverSystem::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  stop_comm_thread_();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiviaRoverSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  /* Reset derived/measured state */
  for (std::size_t i = 0U; i < kNumWheelJoints; ++i)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  consecutive_timeouts_ = 0U;
  fault_stop_.store(false);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MiviaRoverSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  /* Optionally request stop */
  fault_stop_.store(true);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MiviaRoverSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(kNumWheelJoints * 2U);

  for (std::size_t i = 0U; i < kNumWheelJoints; ++i)
  {
    out.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    out.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return out;
}

std::vector<hardware_interface::CommandInterface> MiviaRoverSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(kNumWheelJoints);

  for (std::size_t i = 0U; i < kNumWheelJoints; ++i)
  {
    out.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return out;
}

void MiviaRoverSystem::encoder_callback_(const mivia_rover_can_msgs::msg::EncoderRpms::SharedPtr msg)
{
  EncoderSample s;

  /* Message field order: [front_left, rear_left, front_right, rear_right] */
  s.rpm[0] = msg->front_left;
  s.rpm[1] = msg->rear_left;
  s.rpm[2] = msg->front_right;
  s.rpm[3] = msg->rear_right;

  /* Timestamp always at reception to avoid clock inconsistencies */
  s.stamp_ns = static_cast<std::uint64_t>(comm_node_->now().nanoseconds());
  s.valid = true;

  encoder_buffer_.writeFromNonRT(s);
}

hardware_interface::return_type MiviaRoverSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const EncoderSample * const enc = encoder_buffer_.readFromRT();

  if ((enc == nullptr) || (!enc->valid))
  {
    return hardware_interface::return_type::OK;
  }

  const std::uint64_t now_ns = static_cast<std::uint64_t>(time.nanoseconds());
  const std::uint64_t age_ns = (now_ns >= enc->stamp_ns) ? (now_ns - enc->stamp_ns) : 0ULL;
  const double age_sec = static_cast<double>(age_ns) * kNsToSec;

  if (age_sec > feedback_timeout_sec_)
  {
    consecutive_timeouts_++;

    /* Fail-safe local state */
    for (std::size_t i = 0U; i < kNumWheelJoints; ++i)
    {
      hw_velocities_[i] = 0.0;
    }

    if (consecutive_timeouts_ >= max_consecutive_timeouts_)
    {
      /* Engage stop publishing */
      fault_stop_.store(true);
      return hardware_interface::return_type::ERROR;
    }

    /* Degrade but keep running */
    fault_stop_.store(true);
    return hardware_interface::return_type::OK;
  }

  /* Feedback healthy */
  consecutive_timeouts_ = 0U;
  fault_stop_.store(false);

  /* Integrate positions derived from measured wheel angular velocities */
  double dt = period.seconds();
  if (dt < 0.0)
  {
    dt = 0.0;
  }
  else
  {
    /* no-op */
  }

  for (std::size_t k = 0U; k < kNumWheelJoints; ++k)
  {
    const std::size_t joint_i = wheel_joint_indices_[k];
    const double w = rpm_to_rad_s_(enc->rpm[k]);

    hw_velocities_[joint_i] = w;
    hw_positions_[joint_i] = hw_positions_[joint_i] + (w * dt);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MiviaRoverSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  /* If we are in fault-stop, command buffer can still be updated, but publisher will output zeros */
  CommandSample cmd;
  cmd.valid = true;
  cmd.stamp_ns = static_cast<std::uint64_t>(time.nanoseconds());

  for (std::size_t k = 0U; k < kNumWheelJoints; ++k)
  {
    const std::size_t joint_i = wheel_joint_indices_[k];
    const double rpm = rad_s_to_rpm_(hw_commands_[joint_i]);
    cmd.rpm[k] = clamp_rpm_(rpm, rpm_limit_abs_);
  }

  /* Lock-free double-buffer publish: sequence counter selects active buffer */
  {
    const std::uint32_t seq = cmd_seq_.load(std::memory_order_relaxed);
    const std::size_t next_idx = static_cast<std::size_t>((seq + 1U) & 1U);

    cmd_buf_[next_idx] = cmd;
    cmd_valid_.store(true, std::memory_order_release);

    cmd_seq_.store(seq + 1U, std::memory_order_release);
  }

  return hardware_interface::return_type::OK;
}

void MiviaRoverSystem::start_comm_thread_()
{
  comm_running_.store(true);
  comm_thread_ = std::thread(&MiviaRoverSystem::comm_thread_entry_, this);
}

void MiviaRoverSystem::stop_comm_thread_()
{
  comm_running_.store(false);
  if (comm_thread_.joinable())
  {
    comm_thread_.join();
  }
}

void MiviaRoverSystem::comm_thread_entry_()
{
  double hz = publish_rate_hz_;
  if (hz < 1.0)
  {
    hz = 10.0;
  }
  else
  {
    /* no-op */
  }

  const std::chrono::duration<double> sleep_dur(1.0 / hz);

  while (comm_running_.load())
  {
    if (exec_ != nullptr)
    {
      exec_->spin_some();
    }
    else
    {
      /* no-op */
    }

    if ((reference_pub_ != nullptr) && (comm_node_ != nullptr))
    {
      mivia_rover_can_msgs::msg::Reference out;
      out.header.stamp = comm_node_->now();

      /* Safety: if fault_stop_ asserted, publish zeros */
      if (fault_stop_.load(std::memory_order_relaxed))
      {
        out.front_left = 0;
        out.rear_left = 0;
        out.front_right = 0;
        out.rear_right = 0;

        reference_pub_->publish(out);
      }
      else
      {
        /* Publish last valid command if available */
        if (cmd_valid_.load(std::memory_order_acquire))
        {
          const std::uint32_t seq = cmd_seq_.load(std::memory_order_acquire);
          const std::size_t idx = static_cast<std::size_t>(seq & 1U);

          const CommandSample cmd = cmd_buf_[idx];

          out.front_left = cmd.rpm[0];
          out.rear_left = cmd.rpm[1];
          out.front_right = cmd.rpm[2];
          out.rear_right = cmd.rpm[3];

          reference_pub_->publish(out);
        }
        else
        {
          /* No command yet: publish zeros (safe default) */
          out.front_left = 0;
          out.rear_left = 0;
          out.front_right = 0;
          out.rear_right = 0;

          reference_pub_->publish(out);
        }
      }
    }

    std::this_thread::sleep_for(sleep_dur);
  }
}

}  // namespace mivia_rover_platform

PLUGINLIB_EXPORT_CLASS(mivia_rover_platform::MiviaRoverSystem, hardware_interface::SystemInterface)
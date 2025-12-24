#ifndef DUAL_ARM_HARDWARE_INTERFACE_HPP__
#define DUAL_ARM_HARDWARE_INTERFACE_HPP__

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"

#include "std_msgs/msg/float32.hpp"
#include "ros2_socketcan_msgs/msg/fd_frame.hpp"

#include "dual_arm_hardware_interface/motor_command.hpp"
#include "dual_arm_hardware_interface/visibility_control.h"

using namespace std::chrono_literals;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace dual_arm_hardware_interface 
{

enum integration_level_t : uint8_t
{
  UNDEFINED = 0,
  POSITION = 1,
  EFFORT = 2
};
  
struct MotorConfig 
{
  uint8_t can_id;
  int32_t position_offset;
};

class DualArmHardwareInterface : public hardware_interface::SystemInterface
{
  using FdFrame = ros2_socketcan_msgs::msg::FdFrame;

public:
  DualArmHardwareInterface();
  virtual ~DualArmHardwareInterface() = default;
  
  RCLCPP_SHARED_PTR_DEFINITIONS(DualArmHardwareInterface)

  // explicit DualArmHardwareInterface(rclcpp::Node::SharedPtr node);
  CAN_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  CAN_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CAN_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CAN_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CAN_DRIVER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  CAN_DRIVER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  CAN_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CAN_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CAN_DRIVER_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) override;

  CAN_DRIVER_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) override;

  CAN_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  
  CAN_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  int32_t get_target_pos(double hw_pos_cmd, int32_t pos_offset) const;
  int32_t get_target_vel(double hw_vel_cmd) const;
  int32_t get_target_curr(double hw_eff_cmd) const;

  bool wait_for_subscription(void);

  bool is_configured(void) const;
  void set_configured(bool state);
  bool is_activated(void) const;
  void set_activated(bool state);

private:
  rclcpp::Logger logger_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;

  std::shared_ptr<rclcpp::Node> node_;
  
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

  std::atomic<bool> shutdown_requested_;
  std::thread executor_thread_;

  std::queue<FdFrame::SharedPtr> can_rx_buf_;
  std::mutex can_rx_buf_mutex_;

  std::unordered_map<uint8_t, size_t> can_id_to_index_; // can_id -> index in motor_configs_

  std::string ns_;
  std::string can_interface_;

  std::mutex mutex_;
  std::atomic<bool> activated_{false};
  std::atomic<bool> configured_{false};
  
  std::unordered_map<std::string, size_t> joint_indices_;

  std::vector<MotorConfig> motor_configs_;
  
  std::vector<double> hw_position_commands_;
  std::vector<double> hw_effort_commands_;
  
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;
  
  std::vector<uint16_t> enable_states_;
  std::vector<uint16_t> error_states_;

  std::vector<bool> supports_position_command_;
  std::vector<bool> supports_effort_command_;

  std::mutex control_level_mutex_;
  std::vector<integration_level_t> control_level_;

  rclcpp::CallbackGroup::SharedPtr can_sub_cbg_;
  
  rclcpp::Publisher<FdFrame>::SharedPtr can_pub_;
  rclcpp::Subscription<FdFrame>::SharedPtr can_sub_;

  void process_can_frame(const FdFrame::SharedPtr msg);

  void executor_loop(void);

  bool initialize_can_interface();
  void send_can_frame(uint8_t can_id, uint32_t id_offset, int32_t value = 0);
  void can_frame_cb(const FdFrame::SharedPtr msg);
  bool write_register(uint8_t can_id, uint8_t addr, uint8_t values);
  bool send_IAP_command(uint8_t can_id);
  bool send_enable_command(uint8_t can_id, bool flag);
  bool send_clear_error_command(uint8_t can_id);
  bool send_set_mode_command(uint8_t can_id, uint8_t mode);

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  constexpr static uint8_t READ_CMD = 0x01;
  constexpr static uint8_t WRITE_CMD = 0x02;
  constexpr static uint8_t RX_FRAME_LEN = 12;
  constexpr static size_t MAX_QUEUE_SIZE = 4294967296;
};

} // namespace dual_arm_hardware_interface

#endif // DUAL_ARM_HARDWARE_INTERFACE_HPP__
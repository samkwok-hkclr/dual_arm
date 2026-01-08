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
#include <sched.h>
#include <errno.h>

#include <boost/lockfree/spsc_queue.hpp>

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

#include "dual_arm_msgs/msg/arm_config.hpp"
#include "dual_arm_msgs/msg/arm_status.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace dual_arm_hardware_interface 
{

enum ControlState : uint8_t
{
  UNDEFINED = 0,
  POSITION = 1,
  EFFORT = 2
};

enum ModeSwitchResult : uint8_t
{
  ERROR = 0,
  OK = 1,
  SKIP = 2
};
  
struct MotorConfig 
{
  uint8_t can_id;

  uint16_t position_p;
  uint16_t position_i;
  uint16_t position_d;

  uint16_t velocity_p;
  uint16_t velocity_i;
  uint16_t velocity_d;

  uint16_t current_p;
  uint16_t current_i;
  uint16_t current_d;

  int32_t position_offset;
};

struct MotorStatus 
{
  uint8_t can_id;

  float system_voltage;
  float system_temperature;
  int8_t enable_state;
  int8_t brake_state;

  int16_t error_code;
};

class DualArmHardwareInterface : public hardware_interface::SystemInterface
{
  using FdFrame = ros2_socketcan_msgs::msg::FdFrame;
  using ArmConfig = dual_arm_msgs::msg::ArmConfig;
  using ArmStatus = dual_arm_msgs::msg::ArmStatus;

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
  hardware_interface::return_type read(
    const rclcpp::Time& time, 
    const rclcpp::Duration& period) override;
  
  CAN_DRIVER_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time& time, 
    const rclcpp::Duration& period) override;

  ModeSwitchResult command_mode_switch_verification(
    const std::vector<std::string>& start_interfaces,
    std::vector<ControlState>& new_modes);

  int32_t get_target_pos(double hw_pos_cmd, int32_t pos_offset) const;
  int32_t get_target_vel(double hw_vel_cmd) const;
  int32_t get_target_curr(double hw_eff_cmd) const;
  uint16_t extract_uint16(const rosidl_runtime_cpp::BoundedVector<uint8_t, 64>& data, size_t offset) const;
  int32_t extract_int32(const rosidl_runtime_cpp::BoundedVector<uint8_t, 64>& data, size_t offset) const;

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

  boost::lockfree::spsc_queue<ros2_socketcan_msgs::msg::FdFrame::SharedPtr, boost::lockfree::capacity<8192>> can_frame_queue_;

  std::unordered_map<uint8_t, size_t> can_id_to_index_; // can_id -> index in [motor_configs_, motor_status_]

  std::string ns_;
  std::string can_interface_;

  std::mutex mutex_;
  std::atomic<bool> activated_;
  std::atomic<bool> configured_;
  
  std::unordered_map<std::string, size_t> joint_indices_;

  std::vector<MotorConfig> motor_configs_;
  std::vector<MotorStatus> motor_status_;
  
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
  std::vector<ControlState> control_level_;

  rclcpp::CallbackGroup::SharedPtr timer_cbg;
  rclcpp::CallbackGroup::SharedPtr sub_cbg;

  rclcpp::Publisher<FdFrame>::SharedPtr can_pub_;
  rclcpp::Publisher<ArmConfig>::SharedPtr arm_config_pub_;
  rclcpp::Publisher<ArmStatus>::SharedPtr arm_status_pub_;
  rclcpp::Subscription<FdFrame>::SharedPtr can_sub_;
  rclcpp::TimerBase::SharedPtr read_reg_timer_;
  rclcpp::TimerBase::SharedPtr pub_config_timer_;
  rclcpp::TimerBase::SharedPtr pub_status_timer_;

  void process_can_frame(const FdFrame::SharedPtr msg);
  void process_servo_resp(const FdFrame::SharedPtr msg, const uint8_t can_id);
  void process_status_resp(const FdFrame::SharedPtr msg, const uint8_t can_id);
  void process_iap_resp(const FdFrame::SharedPtr msg, const uint8_t can_id);
  template<typename T>
  void map_register(const uint8_t can_id, uint8_t reg, T value);
  std::optional<int32_t> get_offset(const uint8_t can_id);

  void executor_loop(void);

  void send_ctrl_frame(uint8_t can_id, uint32_t id_offset, int32_t value = 0);
  void can_frame_cb(const FdFrame::SharedPtr msg);
  void read_register_cb();
  void pub_config_cb();
  void pub_status_cb();
  void read_register(uint8_t can_id, uint8_t addr, uint8_t len);
  void write_register(uint8_t can_id, uint8_t addr, uint8_t values);
  void send_IAP_command(uint8_t can_id);
  void send_enable_command(uint8_t can_id, bool flag);
  void send_clear_error_command(uint8_t can_id);
  void send_set_mode_command(uint8_t can_id, uint8_t mode);

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  std::atomic<uint32_t> missed_frames{0};

  const std::vector<MotorRegister> REG_TO_READ = {
    MotorRegister::SYSTEM_VOLTAGE,
    MotorRegister::SYSTEM_TEMP,
    MotorRegister::CURRENT_P,
    MotorRegister::CURRENT_I,
    MotorRegister::CURRENT_D,
    MotorRegister::VELOCITY_P,
    MotorRegister::VELOCITY_I,
    MotorRegister::VELOCITY_D,
    MotorRegister::POSITION_P,
    MotorRegister::POSITION_I,
    MotorRegister::POSITION_D
  };

  static constexpr double POS_SCALE = 0.0001 * (M_PI / 180.0);
  static constexpr double VEL_SCALE = 0.02 * (M_PI / 30.0);
  static constexpr double CURR_SCALE = 0.001;
  static constexpr double OFFSET_SCALE = 0.0001;
  static constexpr double DEG_TO_RAD_SCALE = M_PI / 180.0;

  static constexpr double POS_CVT_SCALE = 180.0 * 10000.0 / M_PI;
  static constexpr double VEL_CVT_SCALE = 30.0 * 500.0 / M_PI;
  static constexpr double CURR_CVT_SCALE = 1000.0;

  constexpr static uint8_t READ_CMD = 0x01;
  constexpr static uint8_t WRITE_CMD = 0x02;
  constexpr static uint8_t RX_FRAME_LEN = 12;
  constexpr static uint8_t IAP_FRAME_LEN = 4;
  constexpr static size_t MAX_QUEUE_SIZE = 4294967296;
};

} // namespace dual_arm_hardware_interface

#endif // DUAL_ARM_HARDWARE_INTERFACE_HPP__
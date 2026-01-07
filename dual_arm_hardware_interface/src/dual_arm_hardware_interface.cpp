#include "dual_arm_hardware_interface/dual_arm_hardware_interface.hpp"

namespace dual_arm_hardware_interface 
{

DualArmHardwareInterface::DualArmHardwareInterface()
  : hardware_interface::SystemInterface(), 
  logger_(rclcpp::get_logger("dual_arm_hardware_interface")), 
  executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{
  shutdown_requested_.store(false);

  executor_thread_ = std::thread(std::bind(&DualArmHardwareInterface::executor_loop, this));
  
  RCLCPP_INFO(logger_, "DualArmHardwareInterface initialized");
}

CallbackReturn DualArmHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) 
    return CallbackReturn::ERROR;
  
  std::lock_guard<std::mutex> lock(mutex_);

  joint_indices_.reserve(info.joints.size());
  supports_position_command_.resize(info.joints.size(), false);
  supports_effort_command_.resize(info.joints.size(), false);
  
  motor_configs_.resize(info.joints.size());
  motor_status_.resize(info.joints.size());
  
  for (size_t i = 0; i < info.joints.size(); ++i) 
  {
    const auto& joint = info.joints[i];
    joint_indices_[joint.name] = i;
    
    try 
    {
      uint8_t can_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("can_id"), nullptr, 16));
      motor_configs_[i].can_id = can_id;
      motor_status_[i].can_id = can_id;
      
      if (joint.parameters.count("position_offset"))
        motor_configs_[i].position_offset = static_cast<int32_t>(std::stol(joint.parameters.at("position_offset"))); 
      else
        motor_configs_[i].position_offset = 0;
      RCLCPP_INFO(logger_, "position_offset read %d", motor_configs_[i].position_offset);

      for (const auto& cmd_if : joint.command_interfaces) 
      {
        if (cmd_if.name == hardware_interface::HW_IF_POSITION) 
        {
          supports_position_command_[i] = true;
        } 
        else if (cmd_if.name == hardware_interface::HW_IF_EFFORT) 
        {
          supports_effort_command_[i] = true;
        }
      }

      RCLCPP_INFO(logger_, "Initialized joint %s (0x%X)", joint.name.c_str(), motor_configs_[i].can_id);
    } 
    catch (const std::exception& e) 
    {
      RCLCPP_ERROR(logger_, "Failed to initialize joint %s: %s", joint.name.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  }

  can_id_to_index_.reserve(info.joints.size());
  for (size_t i = 0; i < motor_configs_.size(); ++i)
  {
    can_id_to_index_[motor_configs_[i].can_id] = i;
    RCLCPP_INFO(logger_, "CAN-ID [0x%X] mapped to index [%zu]", motor_configs_[i].can_id, i);
  }
  
  hw_position_states_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_effort_states_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());

  enable_states_.resize(info.joints.size(), std::numeric_limits<uint16_t>::quiet_NaN());
  error_states_.resize(info.joints.size(), std::numeric_limits<uint16_t>::quiet_NaN());
  
  hw_position_commands_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_effort_commands_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());

  control_level_.resize(info_.joints.size(), ControlState::UNDEFINED);
  
  if (info.hardware_parameters.count("can_interface"))
  {
    can_interface_ = info.hardware_parameters.at("can_interface");
  }
  else
  {
    RCLCPP_ERROR(logger_, "CAN interface does not provided.");
    return CallbackReturn::ERROR;
  }

  if (info.hardware_parameters.count("namespace"))
  {
    ns_ = info.hardware_parameters.at("namespace");
  }
  else
  {
    RCLCPP_ERROR(logger_, "No CAN interface provided.");
    return CallbackReturn::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>("dual_arm_hw_interface_" + info_.name);

  auto initialize_can_interface = [this]() -> bool {
    RCLCPP_INFO(node_->get_logger(), "Using CAN interface %s", can_interface_.c_str());

    const std::string pub_topic = "/" +  ns_ + "/" + can_interface_ + "/to_can_bus_fd";
    const std::string sub_topic = "/" +  ns_ + "/" +  can_interface_ + "/from_can_bus_fd";

    RCLCPP_INFO(node_->get_logger(), "pub_topic %s", pub_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "sub_topic %s", sub_topic.c_str());

    can_pub_ = node_->create_publisher<FdFrame>(
      pub_topic,
      rclcpp::QoS(10).reliable());

    arm_config_pub_ = node_->create_publisher<ArmConfig>(
      "/" + can_interface_ + "/arm_config",
      rclcpp::QoS(10).reliable());

    arm_status_pub_ = node_->create_publisher<ArmStatus>(
      "/" + can_interface_ + "/arm_status",
      rclcpp::QoS(10).reliable());

    can_sub_ = node_->create_subscription<FdFrame>(
      sub_topic,
      rclcpp::QoS(1000).reliable(),
      std::bind(&DualArmHardwareInterface::can_frame_cb, this, std::placeholders::_1));

    read_reg_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DualArmHardwareInterface::read_register_cb, this));

    pub_config_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DualArmHardwareInterface::pub_config_cb, this));

    pub_status_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&DualArmHardwareInterface::pub_status_cb, this));

    return true;
  };

  if (!initialize_can_interface()) 
  {
    RCLCPP_ERROR(logger_, "Failed to initialize CAN interface");
    return CallbackReturn::ERROR;
  }

  if (node_)
  {
    updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
    updater_->setHardwareID(info_.name);

    updater_->add(
      info_.name + "_Status", this,
      &DualArmHardwareInterface::produce_diagnostics);
    RCLCPP_INFO(logger_, "Added a Standard diagnostics to node.");
  }
  else
  {
    RCLCPP_WARN(logger_, "Node is not available. Standard diagnostics will not be published.");
  }

  executor_->add_node(node_);
  RCLCPP_INFO(logger_, "Added node to the executor");

  RCLCPP_INFO(logger_, "Successfully initialize %zu motors", motor_configs_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualArmHardwareInterface::on_configure(const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(logger_, "Start the configure state");

  auto wait_for_subscription = [this]() -> bool {
    const uint8_t MAX_ATTEMPT = 60;
    uint8_t attempt = 0;
    rclcpp::Rate rate(1);

    while (rclcpp::ok())
    {
      if (can_pub_->get_subscription_count() > 0)
        break;
      
      if (attempt >= MAX_ATTEMPT)
        return false;
      
      attempt++;
      RCLCPP_WARN(logger_, "waiting for socketcan subscription");
      rate.sleep();
    }

    return true;
  };

  if (!wait_for_subscription())
  {
    RCLCPP_ERROR(logger_, "Failed to wait for socketcan subscription");
    return CallbackReturn::ERROR;
  }
  
  for (size_t i = 0; i < motor_configs_.size(); ++i) 
  {
    const auto& config = motor_configs_[i];

    if (config.can_id == 0) 
    {
      RCLCPP_ERROR(logger_, "Invalid configuration for motor %zu", i);
      return CallbackReturn::ERROR;
    }
    
    send_IAP_command(config.can_id);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    send_clear_error_command(config.can_id);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    switch (control_level_[i])
    {
      case ControlState::UNDEFINED:
        RCLCPP_INFO(logger_, "UNDEFINED control level on motor ID: 0x%X", config.can_id);
        break;
      case ControlState::POSITION:
        send_set_mode_command(config.can_id, MotorMode::POSITION_MODE);
        break;
      case ControlState::EFFORT:
        send_set_mode_command(config.can_id, MotorMode::EFFORT_MODE);
        break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  RCLCPP_INFO(logger_, "Successfully configured %zu motors", motor_configs_.size());
  set_configured(true);

  for (size_t i = 0; i < hw_position_commands_.size(); ++i) 
  {
    send_ctrl_frame(motor_configs_[i].can_id, CanIdOffset::STATUS_REQ_ID_OFFSET);

    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(10);
    bool position_received = false;
    
    while (rclcpp::ok() && std::chrono::steady_clock::now() - start_time < timeout) 
    {
      if (hw_position_states_[i] != std::numeric_limits<double>::quiet_NaN()) 
      {
        RCLCPP_INFO(logger_, "Initial Position %f", hw_position_states_[i]);
        position_received = true;
        break;
      }

      if (!position_received) 
      {
        RCLCPP_INFO(logger_, "Waiting for motor %zu", i);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    if (!position_received) 
    {
      RCLCPP_ERROR(logger_, "Failed to get position for joint %zu", i);
      return CallbackReturn::ERROR;
    }

    hw_position_commands_[i] = hw_position_states_[i];
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn DualArmHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(logger_, "Start the activate state");

  for (const auto& config : motor_configs_) 
  {
    send_enable_command(config.can_id, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }

  set_activated(true);
  
  RCLCPP_INFO(logger_, "Activated successfully!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(logger_, "Start the deactivate state");

  for (const auto& config : motor_configs_) 
  {
    send_enable_command(config.can_id, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }

  set_activated(false);

  RCLCPP_INFO(logger_, "Deactivated successfully!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualArmHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& /* previous_state */)
{
  can_pub_.reset();
  can_sub_.reset();
  node_.reset();

  shutdown_requested_.store(true);
  
  if (executor_thread_.joinable()) 
    executor_thread_.join();

  RCLCPP_INFO(logger_, "Clean successfully!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualArmHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& /* previous_state */)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DualArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (const auto& joint : info_.joints) 
  {
    size_t index = joint_indices_.at(joint.name);
    
    for (const auto& state_if : joint.state_interfaces) 
    {
      if (state_if.name == hardware_interface::HW_IF_POSITION) 
      {
        state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &hw_position_states_[index]);
      }
      else if (state_if.name == hardware_interface::HW_IF_VELOCITY) 
      {
        state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[index]);
      }
      else if (state_if.name == hardware_interface::HW_IF_EFFORT) 
      {
        state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[index]);
      }
    }
  }
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DualArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (const auto& joint : info_.joints) 
  {
    size_t index = joint_indices_.at(joint.name);

    for (const auto& cmd_if : joint.command_interfaces) 
    {
      if (cmd_if.name == hardware_interface::HW_IF_POSITION) 
      {
        command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[index]);
      }
      else if (cmd_if.name == hardware_interface::HW_IF_EFFORT) 
      {
        command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &hw_effort_commands_[index]);
      }
    }
  }
  
  return command_interfaces;
}

hardware_interface::return_type DualArmHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces)
{
  std::vector<ControlState> new_modes = {};

  switch (command_mode_switch_verification(start_interfaces, new_modes))
  {
    case ModeSwitchResult::ERROR:
      return hardware_interface::return_type::ERROR;
    case ModeSwitchResult::OK:
      // prepare mode switch
      break;
    case ModeSwitchResult::SKIP:
      // not this arm
      return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(control_level_mutex_);

  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        hw_position_commands_[i] = hw_position_states_[i];
        hw_effort_commands_[i] = hw_effort_states_[i];
        control_level_[i] = ControlState::UNDEFINED;  // Revert to undefined

        RCLCPP_DEBUG(logger_, "Revert to undefined control level");
      }
    }
  }

  // Set the new command modes
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (control_level_[i] != ControlState::UNDEFINED)
    {
      RCLCPP_ERROR(logger_, "control level does not reset");
      return hardware_interface::return_type::ERROR;
    }
    control_level_[i] = new_modes[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DualArmHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& /* stop_interfaces */)
{
  std::vector<ControlState> new_modes = {};

  switch (command_mode_switch_verification(start_interfaces, new_modes))
  {
    case ModeSwitchResult::ERROR:
      return hardware_interface::return_type::ERROR;
    case ModeSwitchResult::OK:
      // perform mode switch
      break;
    case ModeSwitchResult::SKIP:
      // not this arm
      return hardware_interface::return_type::OK;
  }
    
  std::lock_guard<std::mutex> lock(control_level_mutex_);

  for (size_t i = 0; i < motor_configs_.size(); ++i) 
  {
    const auto& config = motor_configs_[i];

    switch (control_level_[i])
    {
      case ControlState::UNDEFINED:
        RCLCPP_ERROR(logger_, "UNDEFINED control level on motor ID: 0x%X", config.can_id);
        break;
      case ControlState::POSITION:
        send_set_mode_command(config.can_id, MotorMode::POSITION_MODE);
        RCLCPP_INFO(logger_, "Successfully set position mode on motor ID: 0x%X", config.can_id);
        break;
      case ControlState::EFFORT:
        send_set_mode_command(config.can_id, MotorMode::EFFORT_MODE);
        RCLCPP_INFO(logger_, "Successfully set current mode on motor ID: 0x%X", config.can_id);
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

ModeSwitchResult DualArmHardwareInterface::command_mode_switch_verification(
  const std::vector<std::string>& start_interfaces,
  std::vector<ControlState>& new_modes)
{
  for (std::string key : start_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      std::string merge_position = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
      std::string merge_effort = info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT;

      RCLCPP_DEBUG(logger_, "key:\t%s", key.c_str());
      RCLCPP_DEBUG(logger_, "merge_position:\t%s", merge_position.c_str());
      RCLCPP_DEBUG(logger_, "merge_effort:\t%s", merge_effort.c_str());

      if (key == merge_position)
      {
        new_modes.push_back(ControlState::POSITION);
        RCLCPP_DEBUG(logger_, "new_modes pushed %s", merge_position.c_str());
      }
      if (key == merge_effort)
      {
        new_modes.push_back(ControlState::EFFORT);
        RCLCPP_DEBUG(logger_, "new_modes pushed %s", merge_effort.c_str());
      }
    }
  }

  if (new_modes.size() == 0)
  {
    RCLCPP_DEBUG(logger_, "system name: %s", get_name().c_str());

    auto left_exist = get_name().find("left");
    auto right_exist = get_name().find("right");

    std::string arm;
    if (left_exist != std::string::npos)
    {
      arm = "left";
    }
    else if (right_exist != std::string::npos)
    {
      arm = "right";
    }
    else
    {
      RCLCPP_ERROR(logger_, "requested arm does not exist");
      return ModeSwitchResult::ERROR;
    }

    if (std::all_of(
      start_interfaces.begin(), 
      start_interfaces.end(), 
      [&](const std::string& start_interfaces) { return start_interfaces.find(arm) == std::string::npos; }))
    {
      RCLCPP_DEBUG(logger_, "The request is not this arm");
      return ModeSwitchResult::SKIP;
    }
  }
  else if (new_modes.size() != info_.joints.size())
  {
    RCLCPP_ERROR(logger_, "new mode size does not match, new_modes: [%zu], joints: [%zu]", new_modes.size(), info_.joints.size());
    return ModeSwitchResult::ERROR;
  }

  if (!std::all_of(
    new_modes.begin() + 1, 
    new_modes.end(), 
    [&](ControlState mode) { return mode == new_modes[0]; }))
  {
    RCLCPP_ERROR(logger_, "all new mode are not the same");
    return ModeSwitchResult::ERROR;
  }

  return ModeSwitchResult::OK;
}

hardware_interface::return_type DualArmHardwareInterface::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (!is_configured()) 
    return hardware_interface::return_type::OK;

  FdFrame::SharedPtr msg;
  
  while (can_frame_queue_.pop(msg))
  {
    try
    {
      process_can_frame(msg);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_, "Error processing CAN frame: %s", e.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DualArmHardwareInterface::write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (!is_activated()) 
  {
    RCLCPP_ERROR(logger_, "Not activated");
    return hardware_interface::return_type::ERROR;
  }

  std::lock_guard<std::mutex> lock(control_level_mutex_);

  for (size_t i = 0; i < motor_configs_.size(); ++i) 
  {
    switch (control_level_[i])
    {
      case ControlState::UNDEFINED:
        RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000, "No control level is using the hardware interface!");
        break;
      case ControlState::POSITION:
        if (supports_position_command_[i] && !std::isnan(hw_position_commands_[i])) 
        {
          int32_t target_pos = get_target_pos(hw_position_commands_[i], motor_configs_[i].position_offset);
          send_ctrl_frame(motor_configs_[i].can_id, CanIdOffset::POS_CTRL_ID_OFFSET, target_pos);
        }
        break;
      case ControlState::EFFORT:
        if (supports_effort_command_[i] && !std::isnan(hw_effort_commands_[i])) 
        {
          // int32_t target_current = get_target_curr(hw_effort_states_[i]);
          int32_t target_current = 0;
          send_ctrl_frame(motor_configs_[i].can_id, CanIdOffset::CURR_CTRL_ID_OFFSET, target_current);
          RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 5000, "effort mode in hardware interface!");
        }
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

void DualArmHardwareInterface::send_ctrl_frame(uint8_t can_id, uint32_t id_offset, int32_t value)
{
  auto frame = std::make_unique<FdFrame>(rosidl_runtime_cpp::MessageInitialization::ZERO);
  frame->header.stamp = node_->now();

  switch(id_offset)
  {
    case CanIdOffset::POS_CTRL_ID_OFFSET:
    case CanIdOffset::VEL_CTRL_ID_OFFSET:
    case CanIdOffset::CURR_CTRL_ID_OFFSET:
      frame->id = can_id + id_offset;
      frame->len = 4;
      frame->data.resize(frame->len);
      frame->data[0] = static_cast<uint8_t>(value & 0xFF);
      frame->data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
      frame->data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
      frame->data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
      break;
    case CanIdOffset::STATUS_REQ_ID_OFFSET:
      frame->id = can_id + id_offset;
      frame->len = 0;
      break;
  }

  can_pub_->publish(std::move(frame));
}

void DualArmHardwareInterface::read_register(uint8_t can_id, uint8_t addr, uint8_t len)
{
  auto frame = std::make_unique<FdFrame>(rosidl_runtime_cpp::MessageInitialization::ZERO);
  frame->header.stamp = node_->now();
  frame->id = can_id;
  frame->len = 3;
  frame->data = {READ_CMD, addr, len};

  can_pub_->publish(std::move(frame));
}

void DualArmHardwareInterface::write_register(uint8_t can_id, uint8_t addr, uint8_t values)
{
  auto frame = std::make_unique<FdFrame>(rosidl_runtime_cpp::MessageInitialization::ZERO);
  frame->header.stamp = node_->now();
  frame->id = can_id;
  frame->len = 3;
  frame->data = {WRITE_CMD, addr, values};

  can_pub_->publish(std::move(frame));
}

void DualArmHardwareInterface::send_IAP_command(uint8_t can_id)
{
  write_register(can_id, MotorAddr::IAP_FLAG, 0x00);

  RCLCPP_WARN(logger_, "send IAP command successfully");
}

void DualArmHardwareInterface::send_enable_command(uint8_t can_id, bool flag)
{
  write_register(can_id, MotorAddr::ENABLE_FLAG, flag ? 0x1 : 0x0);

  RCLCPP_WARN(logger_, "send %sable command successfully", flag ? "en" : "dis");
}

void DualArmHardwareInterface::send_clear_error_command(uint8_t can_id)
{
  write_register(can_id, MotorAddr::CLEAR_ERROR, 0x01);
}

void DualArmHardwareInterface::send_set_mode_command(uint8_t can_id, uint8_t mode)
{
  write_register(can_id, MotorAddr::WORK_MODE, mode);
}

void DualArmHardwareInterface::can_frame_cb(const FdFrame::SharedPtr msg)
{
  bool success = can_frame_queue_.push(msg);

  if (!success)
  {
    RCLCPP_ERROR(node_->get_logger(), "CAN RX queue full, dropping frame");
  }
}

void DualArmHardwareInterface::read_register_cb()
{
  if (!is_configured())
    return;

  for (const auto& it : can_id_to_index_)
  {
    for (const auto& reg : REG_TO_READ)
    {
      read_register(it.first, reg, 0x01);
    }
  }
}

void DualArmHardwareInterface::pub_config_cb()
{
  if (!is_configured())
    return;

  if (!arm_config_pub_ || arm_config_pub_->get_subscription_count() == 0)
    return;

  auto arm_config_msg = std::make_unique<ArmConfig>();

  for (const auto& it : can_id_to_index_)
  {
    dual_arm_msgs::msg::MotorConfig msg;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      msg.position_p = motor_configs_[it.second].position_p;
      msg.position_i = motor_configs_[it.second].position_i;
      msg.position_d = motor_configs_[it.second].position_d;

      msg.velocity_p = motor_configs_[it.second].velocity_p;
      msg.velocity_i = motor_configs_[it.second].velocity_i;
      msg.velocity_d = motor_configs_[it.second].velocity_d;

      msg.current_p = motor_configs_[it.second].current_p;
      msg.current_i = motor_configs_[it.second].current_i;
      msg.current_d = motor_configs_[it.second].current_d;

      msg.system_voltage = motor_status_[it.second].system_voltage;
      msg.system_temperature = motor_status_[it.second].system_temperature;
      msg.enable_state = motor_status_[it.second].enable_state;
      msg.brake_state = motor_status_[it.second].brake_state;
      msg.error_code = motor_status_[it.second].error_code;
    }

    arm_config_msg->arm_config.emplace_back(std::move(msg));
  }
    
  arm_config_pub_->publish(std::move(arm_config_msg));
}

void DualArmHardwareInterface::pub_status_cb()
{
  if (!is_configured())
    return;

  if (!arm_status_pub_ || arm_status_pub_->get_subscription_count() == 0)
    return;

  auto arm_status_msg = std::make_unique<ArmStatus>();

  for (const auto& it : can_id_to_index_)
  {
    dual_arm_msgs::msg::MotorStatus msg;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      msg.position = hw_position_states_[it.second];
      msg.velocity = hw_velocity_states_[it.second];
      msg.current = hw_effort_states_[it.second];

      msg.position_cmd = hw_position_commands_[it.second];
    }

    arm_status_msg->arm_status.emplace_back(std::move(msg));
  }
    
  arm_status_pub_->publish(std::move(arm_status_msg));
}

void DualArmHardwareInterface::process_can_frame(const FdFrame::SharedPtr msg)
{
  const uint8_t target_can_id = msg->id & 0xF;
  // RCLCPP_DEBUG(logger_, "CAN ID: 0x%X (base: %d)", msg->id, target_can_id);
  
  switch (msg->id - target_can_id)
  {
    case CanIdOffset::SERVO_RESP_ID_OFFSET:
      process_servo_resp(msg, target_can_id);
      break;
    case CanIdOffset::STATUS_RESP_ID_OFFSET:
      process_status_resp(msg, target_can_id);
      break;
    case CanIdOffset::IAP_FLAG_ID_OFFSET:
      process_iap_resp(msg, target_can_id);
      break;
    case CanIdOffset::STATUS_REQ_ID_OFFSET:
    case CanIdOffset::POS_CTRL_ID_OFFSET:
      break;
    default:
      RCLCPP_ERROR(logger_, "Unhandled CAN message: ID=0x%X (base=%d)", msg->id, target_can_id);
      break;
  }
}

void DualArmHardwareInterface::process_servo_resp(
  const FdFrame::SharedPtr msg, 
  const uint8_t can_id)
{
  if (msg->len < RX_FRAME_LEN)
  {
    RCLCPP_ERROR(logger_, "Invalid frame length: %d (expected >= %d)", msg->len, RX_FRAME_LEN);
    return;
  } 

  const size_t joint_index = can_id_to_index_[can_id];

  int32_t raw_pos = extract_int32(msg->data, 8);
  int32_t raw_vel = extract_int32(msg->data, 4);
  int32_t raw_curr = extract_int32(msg->data, 0);
  uint16_t enable = extract_uint16(msg->data, 12);
  uint16_t err = extract_uint16(msg->data, 14);

  std::optional<int32_t> offset = get_offset(can_id);

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (offset.has_value())
      hw_position_states_[joint_index] = (raw_pos - offset.value()) * POS_SCALE;
    else
      hw_position_states_[joint_index] = raw_pos * POS_SCALE;
    hw_velocity_states_[joint_index] = raw_vel * VEL_SCALE;
    hw_effort_states_[joint_index] = raw_curr * CURR_SCALE;

    motor_status_[joint_index].enable_state = enable;
    motor_status_[joint_index].error_code = err;
  }

  if (err)
  {
    RCLCPP_ERROR(logger_, "CAN ID [0x%X] Error, code: 0x%X, error: %s", 
      can_id, err, error_code_to_str(err).c_str());
  }
}

void DualArmHardwareInterface::process_status_resp(
  const FdFrame::SharedPtr msg, 
  const uint8_t can_id)
{
  if (msg->len < RX_FRAME_LEN)
  {
    RCLCPP_ERROR(logger_, "Invalid frame length: %d (expected >= %d)", msg->len, RX_FRAME_LEN);
    return;
  }

  const size_t joint_index = can_id_to_index_[can_id];
  double pos;
  int32_t raw_pos = extract_int32(msg->data, 8);

  std::optional<int32_t> offset = get_offset(can_id);
  if (offset.has_value())
    pos = (raw_pos - offset.value()) * OFFSET_SCALE;
  else
    pos = raw_pos * OFFSET_SCALE;
  
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hw_position_states_[joint_index] = pos * DEG_TO_RAD_SCALE;

    motor_status_[joint_index].enable_state = msg->data[5];
    motor_status_[joint_index].brake_state = msg->data[6];
  }
}

void DualArmHardwareInterface::process_iap_resp(
  const FdFrame::SharedPtr msg, 
  const uint8_t can_id)
{
  if (msg->len == 3)
  {
    // nothing to do
  }
  else if (msg->len == IAP_FRAME_LEN)
  {
    uint8_t reg = msg->data[1];
    uint16_t value = extract_uint16(msg->data, 2);
    map_register(can_id, reg, value);
  }
}

template<typename T>
void DualArmHardwareInterface::map_register(uint8_t can_id, uint8_t reg, T value)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const size_t joint_index = can_id_to_index_[can_id];
  MotorConfig& config = motor_configs_[joint_index];
  MotorStatus& status = motor_status_[joint_index];

  switch (reg)
  {
    case MotorRegister::SYSTEM_VOLTAGE:
      status.system_voltage = value * 0.01f;
      break;
    case MotorRegister::SYSTEM_TEMP:
      status.system_temperature = value * 0.1f;
      break;
    case MotorRegister::CURRENT_P:
      config.current_p = value;
      break;
    case MotorRegister::CURRENT_I:
      config.current_i = value;
      break;
    case MotorRegister::CURRENT_D:
      config.current_d = value;
      break;
    case MotorRegister::VELOCITY_P:
      config.velocity_p = value;
      break;
    case MotorRegister::VELOCITY_I:
      config.velocity_i = value;
      break;
    case MotorRegister::VELOCITY_D:
      config.velocity_d = value;
      break;
    case MotorRegister::POSITION_P:
      config.position_p = value;
      break;
    case MotorRegister::POSITION_I:
      config.position_i = value;
      break;
    case MotorRegister::POSITION_D:
      config.position_d = value;
      break;
    default:
      RCLCPP_ERROR(logger_, "Unkown");
      break;
  }
}

std::optional<int32_t> DualArmHardwareInterface::get_offset(const uint8_t can_id)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const size_t joint_index = can_id_to_index_[can_id];
  const MotorConfig& config = motor_configs_[joint_index];

  return std::make_optional(config.position_offset);
}

void DualArmHardwareInterface::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Hardware is OK");
}

inline int32_t DualArmHardwareInterface::get_target_pos(double hw_pos_cmd, int32_t pos_offset) const
{
  return static_cast<int32_t>(hw_pos_cmd * POS_CVT_SCALE) + pos_offset;
}

inline int32_t DualArmHardwareInterface::get_target_vel(double hw_vel_cmd) const
{
  return static_cast<int32_t>(hw_vel_cmd * VEL_CVT_SCALE);
}

inline int32_t DualArmHardwareInterface::get_target_curr(double hw_eff_cmd) const
{
  return static_cast<int32_t>(hw_eff_cmd * CURR_CVT_SCALE);
}

inline uint16_t DualArmHardwareInterface::extract_uint16(
  const rosidl_runtime_cpp::BoundedVector<uint8_t, 64>& data, 
  size_t offset) const 
{
  return static_cast<uint16_t>(data[offset] | (data[offset + 1] << 8));
}

inline int32_t DualArmHardwareInterface::extract_int32(
  const rosidl_runtime_cpp::BoundedVector<uint8_t, 64>& data, 
  size_t offset) const
{
  return static_cast<int32_t>(
    data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16) | (data[offset + 3] << 24)
  );
}

void DualArmHardwareInterface::executor_loop(void)
{
  RCLCPP_INFO(logger_, "Start: executor loop");
  
  sched_param sch;
  sch.sched_priority = 80;
  if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) 
  {
    throw std::runtime_error{std::string("failed to set scheduler: ") + std::strerror(errno)};
  }

  while (rclcpp::ok() && !shutdown_requested_.load()) 
  {
    executor_->spin_once();
  }

  RCLCPP_INFO(logger_, "End: executor loop");
}

bool DualArmHardwareInterface::is_configured(void) const
{
  return configured_.load(std::memory_order_acquire);
}

void DualArmHardwareInterface::set_configured(bool state) 
{
  configured_.store(state, std::memory_order_release);
}

bool DualArmHardwareInterface::is_activated(void) const
{
  return activated_.load(std::memory_order_acquire);
}

void DualArmHardwareInterface::set_activated(bool state) 
{
  activated_.store(state, std::memory_order_release);
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  dual_arm_hardware_interface::DualArmHardwareInterface,
  hardware_interface::SystemInterface
)


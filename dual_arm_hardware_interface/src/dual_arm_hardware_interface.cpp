#include "dual_arm_hardware_interface/dual_arm_hardware_interface.hpp"

namespace dual_arm_hardware_interface 
{

DualArmHardwareInterface::DualArmHardwareInterface()
  : hardware_interface::SystemInterface(), 
  logger_(rclcpp::get_logger("dual_arm_hardware_interface")), 
  executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>())
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
  
  for (size_t i = 0; i < info.joints.size(); ++i) 
  {
    const auto& joint = info.joints[i];
    joint_indices_[joint.name] = i;
    
    try 
    {
      motor_configs_[i].can_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("can_id"), nullptr, 16));
      motor_configs_[i].position_offset = joint.parameters.count("position_offset") ? 
        static_cast<int32_t>(std::stol(joint.parameters.at("position_offset"))) : 0 ; 

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

      RCLCPP_ERROR(logger_, "Initialized joint %s (0x%X)", joint.name.c_str(), motor_configs_[i].can_id);
    } 
    catch (const std::exception& e) 
    {
      RCLCPP_ERROR(logger_, "Failed to initialize joint %s: %s", joint.name.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  }

  can_id_to_index_.clear();
  for (size_t i = 0; i < motor_configs_.size(); ++i)
  {
    can_id_to_index_[motor_configs_[i].can_id] = i;
    RCLCPP_ERROR(logger_, "CAN-ID [0x%X] mapped to index [%zu]", motor_configs_[i].can_id, i);
  }
  
  hw_position_states_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_effort_states_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());

  enable_states_.resize(info.joints.size(), std::numeric_limits<uint16_t>::quiet_NaN());
  error_states_.resize(info.joints.size(), std::numeric_limits<uint16_t>::quiet_NaN());
  
  hw_position_commands_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_effort_commands_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());

  control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);
  
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
    RCLCPP_WARN(logger_, "Added a Standard diagnostics to node.");
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

  if (!wait_for_subscription())
  {
    RCLCPP_ERROR(logger_, "Failed to wait for socketcan subscription");
    return CallbackReturn::ERROR;
  }
  
  // 配置电机参数
  for (size_t i = 0; i < motor_configs_.size(); ++i) 
  {
    const auto& config = motor_configs_[i];

    if (config.can_id == 0) 
    {
      RCLCPP_ERROR(logger_, "Invalid configuration for motor %zu", i);
      return CallbackReturn::ERROR;
    }
    
    if (!send_IAP_command(config.can_id)) 
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to enable motor ID: 0x%X", config.can_id);
      return CallbackReturn::ERROR;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!send_clear_error_command(config.can_id)) 
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to clear errors on motor ID: 0x%X", config.can_id);
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    switch (control_level_[i])
    {
      case integration_level_t::UNDEFINED:
        RCLCPP_ERROR(node_->get_logger(), "UNDEFINED control level on motor ID: 0x%X", config.can_id);
        break;
      case integration_level_t::POSITION:
        if (!send_set_mode_command(config.can_id, MotorMode::POSITION_MODE)) 
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to set position mode on motor ID: 0x%X", config.can_id);
          return CallbackReturn::ERROR;
        }
        break;
      case integration_level_t::EFFORT:
        if (!send_set_mode_command(config.can_id, MotorMode::EFFORT_MODE)) 
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to set current mode on motor ID: 0x%X", config.can_id);
          return CallbackReturn::ERROR;
        }
        break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  set_configured(true);

  RCLCPP_INFO(logger_, "Successfully configured %zu motors", motor_configs_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualArmHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(logger_, "Start the activate state");

  if (is_activated())
  {
    RCLCPP_FATAL(logger_, "Double on_activate()");
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < hw_position_commands_.size(); ++i) 
  {
    send_can_frame(motor_configs_[i].can_id, CanIdOffset::STATUS_REQ_ID_OFFSET);

    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(10);
    bool position_received = false;
    
    while (rclcpp::ok() && std::chrono::steady_clock::now() - start_time < timeout) 
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (hw_position_states_[i] != std::numeric_limits<double>::quiet_NaN()) 
        {
          RCLCPP_INFO(logger_, "Initial Position %f", hw_position_states_[i]);
          position_received = true;
          break;
        }
      }

      if (!position_received) 
      {
        RCLCPP_INFO(logger_, "Waiting for motor %zu", i);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    if (!position_received) 
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get position for joint %zu", i);
      return CallbackReturn::ERROR;
    }

    hw_position_commands_[i] = hw_position_states_[i];
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  
  }

  for (const auto& config : motor_configs_) 
  {
    send_enable_command(config.can_id, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  set_activated(true);
  
  RCLCPP_INFO(logger_, "Activated successfully!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */)
{
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
    
    // 检查并添加位置状态接口
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
  // Prepare for new command modes
  std::vector<integration_level_t> new_modes = {};

  for (std::string key : start_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      std::string merge_position = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
      std::string merge_effort = info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT;

      RCLCPP_DEBUG(node_->get_logger(), "key:\t%s", key.c_str());
      RCLCPP_DEBUG(node_->get_logger(), "merge_position:\t%s", merge_position.c_str());
      RCLCPP_DEBUG(node_->get_logger(), "merge_effort:\t%s", merge_effort.c_str());

      if (key == merge_position)
      {
        new_modes.push_back(integration_level_t::POSITION);
        RCLCPP_WARN(node_->get_logger(), "new_modes pushed %s", merge_position.c_str());
      }
      if (key == merge_effort)
      {
        new_modes.push_back(integration_level_t::EFFORT);
        RCLCPP_WARN(node_->get_logger(), "new_modes pushed %s", merge_effort.c_str());
      }
    }
  }

  if (new_modes.size() == 0)
  {
    RCLCPP_WARN(node_->get_logger(), "system name: %s", get_name().c_str());

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
      RCLCPP_ERROR(node_->get_logger(), "requested arm does not exist");
      return hardware_interface::return_type::ERROR;
    }

    if (std::all_of(
      start_interfaces.begin(), 
      start_interfaces.end(), 
      [&](const std::string& start_interfaces) { return start_interfaces.find(arm) == std::string::npos; }))
    {
      RCLCPP_DEBUG(node_->get_logger(), "The request is not this arm");
      return hardware_interface::return_type::OK;
    }
  }
  else if (new_modes.size() != info_.joints.size())
  {
    RCLCPP_ERROR(node_->get_logger(), "new mode size does not match, new_modes: [%zu], joints: [%zu]", new_modes.size(), info_.joints.size());
    return hardware_interface::return_type::ERROR;
  }
  
  if (!std::all_of(
    new_modes.begin() + 1, 
    new_modes.end(), 
    [&](integration_level_t mode) { return mode == new_modes[0]; }))
  {
    RCLCPP_ERROR(node_->get_logger(), "all new mode are not the same");
    return hardware_interface::return_type::ERROR;
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
        control_level_[i] = integration_level_t::UNDEFINED;  // Revert to undefined

        RCLCPP_WARN(node_->get_logger(), "Revert to undefined control level");
      }
    }
  }

  // Set the new command modes
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (control_level_[i] != integration_level_t::UNDEFINED)
    {
      RCLCPP_ERROR(node_->get_logger(), "control level does not reset");
      return hardware_interface::return_type::ERROR;
    }
    control_level_[i] = new_modes[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DualArmHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string>& /* start_interfaces */,
  const std::vector<std::string>& /* stop_interfaces */)
{
  std::lock_guard<std::mutex> lock(control_level_mutex_);

  for (size_t i = 0; i < motor_configs_.size(); ++i) 
  {
    const auto& config = motor_configs_[i];

    switch (control_level_[i])
    {
      case integration_level_t::UNDEFINED:
        RCLCPP_ERROR(node_->get_logger(), "UNDEFINED control level on motor ID: 0x%X", config.can_id);
        break;
      case integration_level_t::POSITION:
        if (!send_set_mode_command(config.can_id, MotorMode::POSITION_MODE)) 
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to set position mode on motor ID: 0x%X", config.can_id);
          return hardware_interface::return_type::ERROR;
        }
        break;
      case integration_level_t::EFFORT:
        if (!send_set_mode_command(config.can_id, MotorMode::EFFORT_MODE)) 
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to set current mode on motor ID: 0x%X", config.can_id);
          return hardware_interface::return_type::ERROR;
        }
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DualArmHardwareInterface::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  std::queue<FdFrame::SharedPtr> temp_queue;
  {
    std::lock_guard<std::mutex> lock(can_rx_buf_mutex_);
    temp_queue.swap(can_rx_buf_);
  }
  
  const size_t frame_count = temp_queue.size();
  if (frame_count > 0) 
  {
    RCLCPP_DEBUG(node_->get_logger(), "Processing %zu CAN frames", frame_count);

    while (!temp_queue.empty()) 
    {
      auto msg = temp_queue.front();
      temp_queue.pop();

      try
      {
        process_can_frame(msg);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(node_->get_logger(), "Error processing CAN frame: %s", e.what());
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DualArmHardwareInterface::write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (!is_activated() || !node_ || !can_pub_) 
  {
    RCLCPP_ERROR(logger_, "Not activated or node is null");
    return hardware_interface::return_type::ERROR;
  }

  std::lock_guard<std::mutex> lock(control_level_mutex_);

  for (size_t i = 0; i < motor_configs_.size(); ++i) 
  {
    switch (control_level_[i])
    {
      case integration_level_t::UNDEFINED:
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "No control level is using the hardware interface!");
        break;
      case integration_level_t::POSITION:
        if (supports_position_command_[i] && !std::isnan(hw_position_commands_[i])) 
        {
          int32_t target_pos = get_target_pos(hw_position_commands_[i], motor_configs_[i].position_offset);
          send_can_frame(motor_configs_[i].can_id, CanIdOffset::POS_CTRL_ID_OFFSET, target_pos);
        }
        break;
      case integration_level_t::EFFORT:
        if (supports_effort_command_[i] && !std::isnan(hw_effort_commands_[i])) 
        {
          // int32_t target_current = get_target_curr(hw_effort_states_[i]);
          int32_t target_current = 0;
          send_can_frame(motor_configs_[i].can_id, CanIdOffset::CUR_CTRL_ID_OFFSET, target_current);
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "effort mode in hardware interface!");
        }
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

bool DualArmHardwareInterface::initialize_can_interface()
{
  RCLCPP_INFO(node_->get_logger(), "Using CAN interface %s", can_interface_.c_str());

  const std::string pub_topic = "/" +  ns_ + "/" + can_interface_ + "/to_can_bus_fd";
  const std::string sub_topic = "/" +  ns_ + "/" +  can_interface_ + "/from_can_bus_fd";

  RCLCPP_INFO(node_->get_logger(), "pub_topic %s", pub_topic.c_str());
  RCLCPP_INFO(node_->get_logger(), "sub_topic %s", sub_topic.c_str());

  can_pub_ = node_->create_publisher<FdFrame>(
    pub_topic,
    rclcpp::QoS(10).reliable());

  can_sub_cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  
  rclcpp::SubscriptionOptions can_sub_options;
  can_sub_options.callback_group = can_sub_cbg_;

  can_sub_ = node_->create_subscription<FdFrame>(
    sub_topic,
    rclcpp::QoS(1000).reliable(),
    std::bind(&DualArmHardwareInterface::can_frame_cb, this, std::placeholders::_1),
    can_sub_options);

  return true;
}

void DualArmHardwareInterface::send_can_frame(uint8_t can_id, uint32_t id_offset, int32_t value)
{
  if (!node_) 
  {
    RCLCPP_ERROR(node_->get_logger(), "send_can_frame error, node_ is null");
    return;
  }

  FdFrame frame(rosidl_runtime_cpp::MessageInitialization::ZERO);

  frame.header.stamp = node_->now();

  if (id_offset == CanIdOffset::POS_CTRL_ID_OFFSET ||
      id_offset == CanIdOffset::VEL_CTRL_ID_OFFSET ||
      id_offset == CanIdOffset::CUR_CTRL_ID_OFFSET) 
  {
    // 单字节命令
    frame.id = can_id + id_offset;
    frame.len = 4;
    frame.data.resize(frame.len);
    frame.data[0] = static_cast<uint8_t>(value & 0xFF);
    frame.data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    frame.data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    frame.data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
  } 
  else if (id_offset == CanIdOffset::STATUS_REQ_ID_OFFSET) 
  {
    // 五字节命令
    frame.id = can_id + id_offset;
    frame.len = 0;
  }

  can_pub_->publish(frame);
}

bool DualArmHardwareInterface::write_register(uint8_t can_id, uint8_t addr, uint8_t values)
{
  if (!node_) 
  {
    RCLCPP_ERROR(node_->get_logger(), "send_can_frame error, node_ is null");
    return false;
  }

  FdFrame frame(rosidl_runtime_cpp::MessageInitialization::ZERO);

  frame.header.stamp = node_->now();
  frame.id = can_id;
  frame.len = 3;

  frame.data = {WRITE_CMD, addr, values};

  can_pub_->publish(std::move(frame));
  return true;
}

bool DualArmHardwareInterface::send_IAP_command(uint8_t can_id)
{
  bool success = write_register(can_id, MotorAddr::IAP_FLAG, 0x00);
  
  if (success)
    RCLCPP_WARN(node_->get_logger(), "send enable command successfully");
  else
    RCLCPP_ERROR(node_->get_logger(), "send enable command failed");
  
  return success;
}

bool DualArmHardwareInterface::send_enable_command(uint8_t can_id, bool flag)
{
  bool success = write_register(can_id, MotorAddr::ENABLE_FLAG, flag ? 0x1 : 0x0);

  if (success)
    RCLCPP_WARN(node_->get_logger(), "send disable command successfully");
  else
    RCLCPP_ERROR(node_->get_logger(), "send disable command failed");

  return success;
}

bool DualArmHardwareInterface::send_clear_error_command(uint8_t can_id)
{
  return write_register(can_id, MotorAddr::CLEAR_ERROR, 0x01);
}

bool DualArmHardwareInterface::send_set_mode_command(uint8_t can_id, uint8_t mode)
{
  return write_register(can_id, MotorAddr::WORK_MODE, mode);
}

void DualArmHardwareInterface::can_frame_cb(const FdFrame::SharedPtr msg)
{
  if (!node_)
  {
    RCLCPP_ERROR(node_->get_logger(), "node_ is null");
    return;
  } 

  {
    std::lock_guard<std::mutex> lock(can_rx_buf_mutex_);

    if (can_rx_buf_.size() < MAX_QUEUE_SIZE) 
    {
      can_rx_buf_.push(msg);
    } 
    else 
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "CAN RX queue full, dropping frame");
    }
  }

  process_can_frame(msg);
}

void DualArmHardwareInterface::process_can_frame(const FdFrame::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_configured()) 
  {
    return;
  }

  if (motor_configs_.empty() || hw_position_states_.empty() || 
      hw_velocity_states_.empty() || hw_effort_states_.empty()) 
  {
    RCLCPP_ERROR(node_->get_logger(), "Hardware interface not fully initialized");
    return;
  }

  if (!node_ || motor_configs_.size() != hw_position_states_.size())
  {
    RCLCPP_ERROR(node_->get_logger(), "Node not initialized!");
    return;
  }

  const uint8_t target_can_id = msg->id & 0xF;
  RCLCPP_DEBUG(node_->get_logger(), "CAN ID: 0x%X (base: %d)", msg->id, target_can_id);

  auto map_it = can_id_to_index_.find(target_can_id);
  if (map_it == can_id_to_index_.end())
  {
    RCLCPP_ERROR(node_->get_logger(), "Unknown CAN ID: 0x%X (base: %d)", msg->id, target_can_id);
    return;
  }

  const size_t joint_index = map_it->second;
  MotorConfig& config = motor_configs_[joint_index];

  auto extract_uint16 = [&](std::size_t offset) -> uint16_t {
    return static_cast<uint16_t>(
      (msg->data[offset + 1] << 8)  |
      msg->data[offset]
    );
  };
  auto extract_int32 = [&](std::size_t offset) -> int32_t {
    return static_cast<int32_t>(
      (msg->data[offset + 3] << 24) |
      (msg->data[offset + 2] << 16) |
      (msg->data[offset + 1] << 8)  |
      msg->data[offset]
    );
  };
  
  if (msg->id == target_can_id + CanIdOffset::SERVO_RESP_ID_OFFSET) 
  {
    if (msg->len < RX_FRAME_LEN)
    {
      RCLCPP_ERROR(node_->get_logger(), "Invalid frame length: %d (expected >= %d)", msg->len, RX_FRAME_LEN);
      return;
    } 

    double pos = extract_int32(8) * 0.0001 - config.position_offset * 0.0001;
    double vel = extract_int32(4) * 0.02;
    double curr = extract_int32(0) / 1.0;
    uint16_t enable = extract_uint16(12);
    uint16_t err = extract_uint16(14);

    hw_position_states_[joint_index] = pos / 180.0 * M_PI;
    hw_velocity_states_[joint_index] = vel / 30.0 * M_PI;
    hw_effort_states_[joint_index] = curr * 0.001; // mA to A
    enable_states_[joint_index] = enable;
    error_states_[joint_index] = err;

    if (error_states_[joint_index])
    {
      RCLCPP_ERROR(node_->get_logger(), "CAN ID [0x%X] Error, code: 0x%X, error: %s", 
        target_can_id, err, error_code_to_str(err).c_str());
    }

    return;
  }
  else if (msg->id == target_can_id + CanIdOffset::STATUS_RESP_ID_OFFSET)
  {
    if (msg->len < RX_FRAME_LEN)
    {
      RCLCPP_ERROR(node_->get_logger(), "Invalid frame length: %d (expected >= %d)", msg->len, RX_FRAME_LEN);
      return;
    }

    double pos = extract_int32(8) * 0.0001 - config.position_offset * 0.0001;
    
    if (pos > 180.0)
    {
      pos = pos - 6.28;
      config.position_offset += 3600000;
    }
    else if (pos < -180.0)
    {
      pos = pos + 6.28;
      config.position_offset -= 3600000;
    }
    hw_position_states_[joint_index] = pos / 180.0 * M_PI;

    return;
  }
  else if (msg->id == target_can_id)
  {
    // nothing to do
    return;
  }
  else if (msg->id == target_can_id + CanIdOffset::STATUS_REQ_ID_OFFSET)
  {
    // nothing to do
    return;
  }
  else if (msg->id == target_can_id + CanIdOffset::POS_CTRL_ID_OFFSET)
  {
    // nothing to do
    return;
  }
  else if (msg->id == target_can_id + CanIdOffset::IAP_FLAG_ID_OFFSET)
  {
    // nothing to do
    return;
  }

  RCLCPP_ERROR(node_->get_logger(), "Unhandled CAN message: ID=0x%X (base=%d)", msg->id, target_can_id);
}

void DualArmHardwareInterface::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Hardware is OK");
}

int32_t DualArmHardwareInterface::get_target_pos(double hw_pos_cmd, int32_t pos_offset) const
{
  return static_cast<int32_t>(hw_pos_cmd / M_PI * 180.0 * 10000.0) + pos_offset;
}

int32_t DualArmHardwareInterface::get_target_vel(double hw_vel_cmd) const
{
  return static_cast<int32_t>(hw_vel_cmd / M_PI * 30.0 * 500.0);
}

int32_t DualArmHardwareInterface::get_target_curr(double hw_eff_cmd) const
{
  return static_cast<int32_t>(hw_eff_cmd * 1000.0); // A to mA
}

void DualArmHardwareInterface::executor_loop(void)
{
  RCLCPP_INFO(logger_, "Start: executor loop");

  while (rclcpp::ok() && !shutdown_requested_.load()) 
  {
    executor_->spin_once();
  }

  RCLCPP_INFO(logger_, "End: executor loop");
}

bool DualArmHardwareInterface::wait_for_subscription(void)
{
  const uint8_t MAX_ATTEMPT = 60;
  uint8_t attempt = 0;

  rclcpp::Rate rate(1);

  while (rclcpp::ok())
  {
    if (can_pub_->get_subscription_count() > 0)
      break;
    
    if (attempt > MAX_ATTEMPT)
      return false;
    
    attempt++;
    RCLCPP_WARN(logger_, "waiting for socketcan subscription");
    rate.sleep();
  }

  return true;
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


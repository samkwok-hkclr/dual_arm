#include "dual_arm_hardware_interface/dual_arm_hardware_interface.hpp"

namespace dual_arm_hardware_interface 
{

DualArmHardwareInterface::DualArmHardwareInterface()
  : hardware_interface::SystemInterface()
  , logger_(rclcpp::get_logger("dual_arm_hardware_interface"))
  , executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>())
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

  // 初始化关节索引映射和支持的接口标志
  joint_indices_.reserve(info.joints.size());
  supports_position_command_.resize(info.joints.size(), false);
  supports_velocity_command_.resize(info.joints.size(), false);
  supports_effort_command_.resize(info.joints.size(), false);
  
  // 初始化电机配置
  motor_configs_.resize(info.joints.size());
  
  // 从硬件参数加载配置
  for (size_t i = 0; i < info.joints.size(); ++i) 
  {
    const auto& joint = info.joints[i];
    joint_indices_[joint.name] = i;
    
    try 
    {
      motor_configs_[i].can_id = static_cast<uint32_t>(std::stoi(joint.parameters.at("can_id"), nullptr, 16));
      motor_configs_[i].position_offset = joint.parameters.count("position_offset") ? 
        static_cast<int32_t>(std::stol(joint.parameters.at("position_offset"))) : 0 ; 

      RCLCPP_INFO(logger_, "position_offset read %d", motor_configs_[i].position_offset);

      // 检查支持的命令接口
      for (const auto& cmd_if : joint.command_interfaces) 
      {
        if (cmd_if.name == hardware_interface::HW_IF_POSITION) 
        {
          supports_position_command_[i] = true;
        } 
        else if (cmd_if.name == hardware_interface::HW_IF_VELOCITY) 
        {
          supports_velocity_command_[i] = true;
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
  
  // 初始化状态和命令变量
  hw_position_states_.resize(info.joints.size(), std::numeric_limits<double>::max());
  hw_velocity_states_.resize(info.joints.size(), 0.0);
  hw_effort_states_.resize(info.joints.size(), 0.0);
  
  hw_position_commands_.resize(info.joints.size(), 0.0);
  hw_velocity_commands_.resize(info.joints.size(), 0.0);
  hw_effort_commands_.resize(info.joints.size(), 0.0);
  
  if (info.hardware_parameters.count("can_interface"))
  {
    can_interface_ = info.hardware_parameters.at("can_interface");
  }
  else
  {
    RCLCPP_ERROR(logger_, "No CAN interface provided.");
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

  auto create_wait_for_subscription = [this](uint32_t max_attempts, std::chrono::milliseconds check_interval) -> std::function<bool(void)> {
    return [this, max_attempts, check_interval]() -> bool {
      uint32_t attempt = 0;
      rclcpp::Rate rate(check_interval);
      
      while (rclcpp::ok()) 
      {
        if (can_pub_ && can_pub_->get_subscription_count() > 0)
          return true;
        
        if (attempt >= max_attempts)
          return false;
        
        attempt++;
        RCLCPP_WARN(logger_, "waiting for socketcan subscription");
        rate.sleep();
      }
      
      return false;
    };
  };

  auto wait_func = create_wait_for_subscription(60, std::chrono::seconds(1));

  if (!wait_func())
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

    // 清除错误
    if (!send_clear_error_command(config.can_id)) 
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to clear errors on motor ID: 0x%X", config.can_id);
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 设置控制模式
    if (supports_position_command_[i]) 
    {
      if (!send_set_mode_command(config.can_id, MotorMode::POSITION_MODE)) 
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set position mode on motor ID: 0x%X", config.can_id);
        return CallbackReturn::ERROR;
      }
    } 
    else if (supports_velocity_command_[i]) 
    {
      if (!send_set_mode_command(config.can_id, MotorMode::VELOCITY_MODE)) 
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set velocity mode on motor ID: 0x%X", config.can_id);
        return CallbackReturn::ERROR;
      }
    } 
    else if (supports_effort_command_[i]) 
    {
      if (!send_set_mode_command(config.can_id, MotorMode::EFFORT_MODE)) 
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set current mode on motor ID: 0x%X", config.can_id);
        return CallbackReturn::ERROR;
      }
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

    // 等待响应（带超时）
    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(10);
    bool position_received = false;
    
    while (rclcpp::ok() && std::chrono::steady_clock::now() - start_time < timeout) 
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (hw_position_states_[i] != std::numeric_limits<double>::max()) 
        {
          RCLCPP_INFO(logger_, "Initial Position %f", hw_position_states_[i]);
          position_received = true;
          break;
        }
      }

      if (!position_received) 
      {
        // 在等待期间，让出CPU，以便后台线程可以运行
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

  RCLCPP_INFO(logger_, "Clean up successfully!");
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
      else if (cmd_if.name == hardware_interface::HW_IF_VELOCITY) 
      {
        command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[index]);
      }
      else if (cmd_if.name == hardware_interface::HW_IF_EFFORT) 
      {
        command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &hw_effort_commands_[index]);
      }
    }
  }
  
  return command_interfaces;
}

hardware_interface::return_type DualArmHardwareInterface::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  std::queue<CanFrameStamped> temp_queue;
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
      auto frame = temp_queue.front();
      temp_queue.pop();

      try
      {
        process_can_frame(frame.frame);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(node_->get_logger(), "Error processing CAN frame: %s", e.what());
      }
    }
  }

  // for (const auto& config : motor_configs_) {
  //   send_can_frame(config.can_id, CanIdOffset::STATUS_REQ_ID_OFFSET, 0.0);
  // }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DualArmHardwareInterface::write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (!is_activated() || !can_pub_) 
  {
    RCLCPP_ERROR(logger_, "Not activated or can_pub_ is null");
    return hardware_interface::return_type::ERROR;
  }

  // return hardware_interface::return_type::OK;
  for (size_t i = 0; i < motor_configs_.size(); ++i) 
  {
    if (supports_position_command_[i] && !std::isnan(hw_position_commands_[i])) 
    {
      // 位置控制模式
      int32_t target_pos = get_target_pos(hw_position_commands_[i], motor_configs_[i].position_offset);
      send_can_frame(motor_configs_[i].can_id, CanIdOffset::POS_CTRL_ID_OFFSET, target_pos);
    }
    else if (supports_velocity_command_[i] && !std::isnan(hw_velocity_commands_[i])) 
    {
      // 速度控制模式
      int32_t target_vel = get_target_vel(hw_velocity_commands_[i]);
      send_can_frame(motor_configs_[i].can_id, CanIdOffset::VEL_CTRL_ID_OFFSET, target_vel);
    }
    else if (supports_effort_command_[i] && !std::isnan(hw_effort_commands_[i])) 
    {
      // 力矩控制模式
      int32_t target_current = get_target_curr(hw_effort_commands_[i]);
      send_can_frame(motor_configs_[i].can_id, CanIdOffset::CUR_CTRL_ID_OFFSET, target_current);
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

void DualArmHardwareInterface::send_can_frame(uint32_t can_id, uint32_t id_offset, int32_t value)
{
  if (!node_) 
  {
    RCLCPP_ERROR(node_->get_logger(), "send_can_frame error,node_ is null");
    return;
  }

  if (!can_pub_)
  {
    RCLCPP_WARN(node_->get_logger(), "CAN publisher does not initialized!");
    return;
  }

  if (can_pub_->get_subscription_count() == 0)
  {
    RCLCPP_WARN(node_->get_logger(), "No CAN subscription!");
    return;
  }

  auto frame = std::make_unique<FdFrame>();

  frame->header.stamp = node_->now();

  if (id_offset == CanIdOffset::POS_CTRL_ID_OFFSET ||
      id_offset == CanIdOffset::VEL_CTRL_ID_OFFSET ||
      id_offset == CanIdOffset::CUR_CTRL_ID_OFFSET) 
  {
    frame->id = can_id + id_offset;
    frame->len = 4;
    frame->data.resize(frame->len);
    frame->data[0] = static_cast<uint8_t>(value & 0xFF);
    frame->data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    frame->data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    frame->data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
  } 
  else if (id_offset == CanIdOffset::STATUS_REQ_ID_OFFSET) 
  {
    frame->id = can_id + id_offset;
    frame->len = 0;
  }

  can_pub_->publish(std::move(frame));
}

bool DualArmHardwareInterface::write_register(uint32_t can_id, uint8_t addr, uint8_t values)
{
  if (!node_) 
  {
    RCLCPP_ERROR(node_->get_logger(), "send_can_frame error,node_ is null");
    return false;
  }

  if (!can_pub_)
    return false;

  if (can_pub_->get_subscription_count() == 0)
  {
    RCLCPP_WARN(node_->get_logger(), "No CAN subscription!");
    return true;
  }

  auto frame = std::make_unique<FdFrame>();

  // 使用节点时钟获取时间戳
  frame->header.stamp = node_->now();
  frame->id = can_id;
  frame->len = 3;
  frame->data.resize(frame->len);
  frame->data[0] = WRITE_CMD;
  frame->data[1] = addr;
  frame->data[2] = values;

  can_pub_->publish(std::move(frame));
  return true;
}

bool DualArmHardwareInterface::send_IAP_command(uint32_t can_id)
{
  bool success = write_register(can_id, MotorADDR::IAP_FLAG, 0x00);
  
  if (success)
    RCLCPP_WARN(node_->get_logger(), "send enable command successfully");
  else
    RCLCPP_ERROR(node_->get_logger(), "send enable command failed");
  
  return success;
}

bool DualArmHardwareInterface::send_enable_command(uint32_t can_id, bool flag)
{
  bool success = write_register(can_id, MotorADDR::ENABLE_FLAG, flag ? 0x1 : 0x0);

  if (success)
    RCLCPP_WARN(node_->get_logger(), "send disable command successfully");
  else
    RCLCPP_ERROR(node_->get_logger(), "send disable command failed");

  return success;
}

bool DualArmHardwareInterface::send_clear_error_command(uint32_t can_id)
{
  return write_register(can_id, MotorADDR::CLEAR_ERROR, 0x01);
}

bool DualArmHardwareInterface::send_set_mode_command(uint32_t can_id, uint8_t mode)
{
  return write_register(can_id, MotorADDR::WORK_MODE, mode);
}

void DualArmHardwareInterface::can_frame_cb(const FdFrame::SharedPtr msg)
{
  if (!node_)
  {
    RCLCPP_ERROR(node_->get_logger(), "node_ is null");
    return;
  } 

  CanFrameStamped frame_stamped;
  frame_stamped.stamp = node_->now();
  frame_stamped.frame = *msg;

  {
    std::lock_guard<std::mutex> lock(can_rx_buf_mutex_);
    if (can_rx_buf_.size() < MAX_QUEUE_SIZE) 
    {
      can_rx_buf_.push(frame_stamped);
    } 
    else 
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "CAN RX queue full, dropping frame");
    }
  }

  process_can_frame(frame_stamped.frame);
}

void DualArmHardwareInterface::process_can_frame(const FdFrame& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_configured()) 
  {
    return;
  }

  if (!node_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Node not initialized!");
    return;
  }

  const uint8_t target_can_id = msg.id & 0xF;
  RCLCPP_DEBUG(node_->get_logger(), "CAN ID: 0x%X (base: %d)", msg.id, target_can_id);

  auto map_it = can_id_to_index_.find(target_can_id);
  if (map_it == can_id_to_index_.end())
  {
    RCLCPP_ERROR(node_->get_logger(), "Unknown CAN ID: 0x%X (base: %d)", msg.id, target_can_id);
    return;
  }

  const size_t joint_index = map_it->second;
  MotorConfig& config = motor_configs_[joint_index];

  // Helper to extract int32_t from little-endian bytes
  auto extract_int32 = [&](std::size_t offset) -> int32_t {
    return static_cast<int32_t>(
      (msg.data[offset + 3] << 24) |
      (msg.data[offset + 2] << 16) |
      (msg.data[offset + 1] << 8)  |
      msg.data[offset]
    );
  };

  if (msg.id == target_can_id + CanIdOffset::SERVO_RESP_ID_OFFSET) 
  {
    if (msg.len < RX_FRAME_LEN)
    {
      RCLCPP_ERROR(node_->get_logger(), "Invalid frame length: %d (expected >= %d)", msg.len, RX_FRAME_LEN);
      return;
    } 

    float pos = extract_int32(8) * 0.0001 - config.position_offset * 0.0001;
    float vel = extract_int32(4) * 0.02;
    float curr = extract_int32(0) / 1.0;
        
    hw_position_states_[joint_index] = pos / 180.0 * M_PI;
    hw_velocity_states_[joint_index] = vel / 30.0 * M_PI;
    hw_effort_states_[joint_index] = curr * 0.001 ; // mA to A

    return;
  }
  else if (msg.id == target_can_id + CanIdOffset::STATUS_RESP_ID_OFFSET)
  {
    if (msg.len < RX_FRAME_LEN)
    {
      RCLCPP_ERROR(node_->get_logger(), "Invalid frame length: %d (expected >= %d)", msg.len, RX_FRAME_LEN);
      return;
    }

    float pos = extract_int32(8) * 0.0001 - config.position_offset * 0.0001;
    
    if (pos > 180.0)
    {
      pos -= 2.0 * M_PI;
      config.position_offset += 3600000;
    }
    else if (pos < -180.0)
    {
      pos += 2.0 * M_PI;
      config.position_offset -= 3600000;
    }
    hw_position_states_[joint_index] = pos / 180.0 * M_PI;

    return;
  }
  else if (msg.id == target_can_id)
  {
    RCLCPP_WARN(node_->get_logger(), "This CAN message should be filtered by ros2_socket_can: frame id=0x%X", msg.id);
    return;
  }
  else if (msg.id == target_can_id + CanIdOffset::STATUS_REQ_ID_OFFSET)
  {
    RCLCPP_WARN(node_->get_logger(), "This CAN message should be filtered by ros2_socket_can: frame id=0x%X", msg.id);
    return;
  }
  else if (msg.id == target_can_id + CanIdOffset::POS_CTRL_ID_OFFSET)
  {
    RCLCPP_WARN(node_->get_logger(), "This CAN message should be filtered by ros2_socket_can: frame id=0x%X", msg.id);
    return;
  }
  else if (msg.id == target_can_id + CanIdOffset::IAP_FLAG_ID_OFFSET)
  {
    RCLCPP_WARN(node_->get_logger(), "This CAN message should be filtered by ros2_socket_can: frame id=0x%X", msg.id);
    return;
  }

  RCLCPP_ERROR(node_->get_logger(), "Unhandled CAN message: ID=0x%X (base=%d)", msg.id, target_can_id);
}

void DualArmHardwareInterface::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Hardware is OK");
}

int32_t DualArmHardwareInterface::get_target_pos(double hw_pos_cmd, int32_t pos_offset) const
{
  return static_cast<int32_t>(hw_pos_cmd / M_PI * 180 * 10000) + pos_offset;
}

int32_t DualArmHardwareInterface::get_target_vel(double hw_vel_cmd) const
{
  return static_cast<int32_t>(hw_vel_cmd / M_PI * 30 * 500);
}

int32_t DualArmHardwareInterface::get_target_curr(double hw_eff_cmd) const
{
  return static_cast<int32_t>(hw_eff_cmd * 1000); // A to mA
}

void DualArmHardwareInterface::executor_loop(void)
{
  RCLCPP_INFO(logger_, "Start the executor loop");

  while (rclcpp::ok() && !shutdown_requested_.load()) 
  {
    executor_->spin_once();
  }

  RCLCPP_INFO(logger_, "End the executor loop");
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


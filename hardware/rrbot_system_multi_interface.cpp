#include "ros2_control_blue_reach_5/rrbot_system_multi_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_blue_reach_5
{
  hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.serial_port_ = info_.hardware_parameters["serial_port"];
    cfg_.state_update_freq_ = std::stoi(info_.hardware_parameters["state_update_frequency"]);
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    cfg_.hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    cfg_.hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    cfg_.hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);

    hw_joint_structs_.reserve(info_.joints.size());
    control_level_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      std::string device_id_value = joint.parameters.at("device_id");
      double default_position = stod(joint.parameters.at("home"));

      uint8_t device_id = static_cast<uint8_t>(std::stoul(device_id_value, nullptr, 16));

      RCLCPP_INFO(
          rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Device with id %u found", static_cast<unsigned int>(device_id));
      RCLCPP_INFO(
          rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Device default position is %f", default_position);

      Joint::State initialState{default_position, 0.0, 0.0};
      hw_joint_structs_.emplace_back(joint.name, device_id, initialState);
      // RRBotSystemMultiInterface has exactly 3 state interfaces
      // and 3 command interfaces on each joint
      if (joint.command_interfaces.size() != 3)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
            "Joint '%s' has %zu command interfaces. 3 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
            joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
            joint.command_interfaces[0].name == custom_hardware_interface::HW_IF_CURRENT))
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
            "Joint '%s' has %s command interface. Expected %s, %s, %s or %s.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY, custom_hardware_interface::HW_IF_CURRENT, hardware_interface::HW_IF_ACCELERATION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 4)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
            "Joint '%s'has %zu state interfaces. 3 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
            joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
            joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION ||
            joint.state_interfaces[0].name == custom_hardware_interface::HW_IF_CURRENT))
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
            "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY, custom_hardware_interface::HW_IF_CURRENT);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_configure(const rclcpp_lifecycle::State &)
  {
    // Start the driver
    try
    {
      driver_.start(cfg_.serial_port_, 5, false);
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL( // NOLINT
          rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
          "Failed to configure the serial driver for the AlphaHardware system interface.");

      return hardware_interface::CallbackReturn::ERROR;
    }

    // Register callbacks for joint states
    driver_.subscribe(
        alpha::driver::PacketId::PacketID_POSITION,
        [this](const alpha::driver::Packet &packet) -> void
        { updatePositionCb(packet, hw_joint_structs_); });

    driver_.subscribe(
        alpha::driver::PacketId::PacketID_VELOCITY,
        [this](const alpha::driver::Packet &packet) -> void
        { updateVelocityCb(packet, hw_joint_structs_); });

    driver_.subscribe(
        alpha::driver::PacketId::PacketID_CURRENT,
        [this](const alpha::driver::Packet &packet) -> void
        { updateCurrentCb(packet, hw_joint_structs_); });

    // Start a thread to request state updates
    running_.store(true);
    state_request_worker_ = std::thread(&RRBotSystemMultiInterfaceHardware::pollState, this, cfg_.state_update_freq_);

    RCLCPP_INFO( // NOLINT
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Successfully configured the RRBotSystemMultiInterfaceHardware system interface for serial communication!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO( // NOLINT
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Shutting down the AlphaHardware system interface.");

    running_.store(false);
    state_request_worker_.join();
    driver_.stop();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  RRBotSystemMultiInterfaceHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_structs_[i].current_state_.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_structs_[i].current_state_.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_joint_structs_[i].current_state_.acceleration));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_joint_structs_[i].current_state_.current));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  RRBotSystemMultiInterfaceHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_structs_[i].command_state_.position));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_structs_[i].command_state_.velocity));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_joint_structs_[i].command_state_.current));
    }

    return command_interfaces;
  }

  hardware_interface::return_type RRBotSystemMultiInterfaceHardware::prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces)
  {
    // Prepare for new command modes
    std::vector<mode_level_t> new_modes = {};
    for (std::string key : start_interfaces)
    {
      for (std::size_t i = 0; i < info_.joints.size(); i++)
      {
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
        {
          new_modes.push_back(mode_level_t::MODE_POSITION);
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
        {
          new_modes.push_back(mode_level_t::MODE_VELOCITY);
        }
        if (key == info_.joints[i].name + "/" + custom_hardware_interface::HW_IF_CURRENT)
        {
          new_modes.push_back(mode_level_t::MODE_CURRENT);
        }
      }
    }
    // Example criteria: All joints must be given new command mode at the same time
    if (new_modes.size() != info_.joints.size())
    {
      return hardware_interface::return_type::ERROR;
    }
    // Example criteria: All joints must have the same command mode
    if (!std::all_of(
            new_modes.begin() + 1, new_modes.end(),
            [&](mode_level_t mode)
            { return mode == new_modes[0]; }))
    {
      return hardware_interface::return_type::ERROR;
    }

    // Stop motion on all relevant joints that are stopping
    for (std::string key : stop_interfaces)
    {
      for (std::size_t i = 0; i < info_.joints.size(); i++)
      {
        if (key.find(info_.joints[i].name) != std::string::npos)
        {
          hw_joint_structs_[i].command_state_.velocity = 0;
          hw_joint_structs_[i].command_state_.current = 0;
          control_level_[i] = mode_level_t::MODE_DISABLE; // Revert to undefined
        }
      }
    }
    // Set the new command modes
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (control_level_[i] != mode_level_t::MODE_DISABLE)
      {
        // Something else is using the joint! Abort!
        return hardware_interface::return_type::ERROR;
      }
      control_level_[i] = new_modes[i];
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Activating... please wait...");

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

      if (std::isnan(hw_joint_structs_[i].current_state_.position) || hw_joint_structs_[i].current_state_.position == 0)
      {
        hw_joint_structs_[i].current_state_.position = hw_joint_structs_[i].default_state_.position;
      }
      if (std::isnan(hw_joint_structs_[i].current_state_.velocity))
      {
        hw_joint_structs_[i].current_state_.velocity = 0;
      }
      if (std::isnan(hw_joint_structs_[i].current_state_.current))
      {
        hw_joint_structs_[i].current_state_.current = 0;
      }
      if (std::isnan(hw_joint_structs_[i].current_state_.acceleration))
      {
        hw_joint_structs_[i].current_state_.acceleration = 0;
      }
      if (std::isnan(hw_joint_structs_[i].command_state_.position))
      {
        hw_joint_structs_[i].command_state_.position = 0;
      }
      if (std::isnan(hw_joint_structs_[i].command_state_.velocity))
      {
        hw_joint_structs_[i].command_state_.velocity = 0;
      }
      if (std::isnan(hw_joint_structs_[i].command_state_.current))
      {
        hw_joint_structs_[i].command_state_.current = 0;
      }
      control_level_[i] = mode_level_t::MODE_DISABLE;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "System successfully activated! %u",
        control_level_[0]);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Deactivating... please wait...");

    for (int i = 0; i < cfg_.hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
          rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "%.1f seconds left...",
          cfg_.hw_stop_sec_ - i);
    }

    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Successfully deactivated!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type RRBotSystemMultiInterfaceHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    const std::lock_guard<std::mutex> lock(access_async_states_);

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      // RCLCPP_INFO(
      //     rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
      //     "Got pos: %.5f joint %s!",
      //     hw_joint_structs_[i].async_state_.position,
      //     info_.joints[i].name.c_str());

      switch (control_level_[i])
      {
      case mode_level_t::MODE_DISABLE:
        // RCLCPP_INFO(
        //   rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        //   "Nothing is using the hardware interface!");
        return hardware_interface::return_type::OK;
        break;
      case mode_level_t::MODE_POSITION:
        hw_joint_structs_[i].current_state_.acceleration = 0;
        hw_joint_structs_[i].current_state_.current = 0;
        hw_joint_structs_[i].current_state_.velocity = 0;
        hw_joint_structs_[i].current_state_.position +=
            (hw_joint_structs_[i].command_state_.position - hw_joint_structs_[i].current_state_.position) / cfg_.hw_slowdown_;
        break;
      case mode_level_t::MODE_VELOCITY:
        hw_joint_structs_[i].current_state_.acceleration = 0;
        hw_joint_structs_[i].current_state_.current = 0;
        hw_joint_structs_[i].current_state_.velocity = hw_joint_structs_[i].command_state_.velocity;
        hw_joint_structs_[i].current_state_.position += (hw_joint_structs_[i].current_state_.velocity * period.seconds()) / cfg_.hw_slowdown_;
        break;
      case mode_level_t::MODE_CURRENT:
        hw_joint_structs_[i].current_state_.current = hw_joint_structs_[i].command_state_.current;
        hw_joint_structs_[i].current_state_.acceleration = hw_joint_structs_[i].command_state_.current / 2; // dummy
        hw_joint_structs_[i].current_state_.velocity += (hw_joint_structs_[i].current_state_.acceleration * period.seconds()) / cfg_.hw_slowdown_;
        hw_joint_structs_[i].current_state_.position += (hw_joint_structs_[i].current_state_.velocity * period.seconds()) / cfg_.hw_slowdown_;
        break;
      case mode_level_t::MODE_STANDBY:
        hw_joint_structs_[i].current_state_.current = hw_joint_structs_[i].command_state_.current;
        hw_joint_structs_[i].current_state_.acceleration = hw_joint_structs_[i].command_state_.current / 2; // dummy
        hw_joint_structs_[i].current_state_.velocity += (hw_joint_structs_[i].current_state_.acceleration * period.seconds()) / cfg_.hw_slowdown_;
        hw_joint_structs_[i].current_state_.position += (hw_joint_structs_[i].current_state_.velocity * period.seconds()) / cfg_.hw_slowdown_;
        break;
      }
      // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
      // RCLCPP_INFO(
      //   rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
      //   "Got pos: %.5f, vel: %.5f, acc: %.5f, cur: %.5f for joint %s!",
      //   hw_joint_structs_[i].position_state_,
      //   hw_joint_structs_[i].velocity_state_,
      //   hw_joint_structs_[i].acceleration_state_,
      //   hw_joint_structs_[i].current_state_, info_.joints[i].name.c_str());
      // END: This part here is for exemplary purposes - Please do not copy to your production code
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RRBotSystemMultiInterfaceHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    // for (std::size_t i = 0; i < info_.joints.size(); i++)
    // {
    //   // Simulate sending commands to the hardware
    //   RCLCPP_INFO(
    //     rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
    //     "Got the commands pos: %.5f, vel: %.5f, cur: %.5f for joint %s, control_lvl:%u",
    //     hw_joint_structs_[i].position_command_,
    //     hw_joint_structs_[i].velocity_command_,
    //     hw_joint_structs_[i].current_command_,
    //     info_.joints[i].name.c_str(),
    //     control_level_[i]);
    // }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
  }

  void RRBotSystemMultiInterfaceHardware::updatePositionCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_structs_ref)
  {
    if (packet.getData().size() != 4)
    {
      return;
    }

    float position;
    std::memcpy(&position, &packet.getData()[0], sizeof(position)); // NOLINT

    // Convert from mm to m if the message is from the jaws
    position = packet.getDeviceId() == alpha::driver::DeviceId::kLinearJaws ? position / 1000 : position;

    const std::lock_guard<std::mutex> lock(access_async_states_);
    // RCLCPP_INFO(
    //     rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "async position is %f", position);

    auto deviceId = static_cast<uint8_t>(packet.getDeviceId()); // Cast the device ID to uint8_t

    auto it = std::find_if(hw_joint_structs_ref.begin(), hw_joint_structs_ref.end(),
                           [deviceId](const Joint &joint)
                           { return joint.device_id == deviceId; });

    if (it != hw_joint_structs_ref.end())
    {
      it->async_state_.position = position;
    }
  }

  void RRBotSystemMultiInterfaceHardware::updateVelocityCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_structs_ref)
  {
    if (packet.getData().size() != 4)
    {
      return;
    }

    float velocity;
    std::memcpy(&velocity, &packet.getData()[0], sizeof(velocity)); // NOLINT

    // Convert from mm/s to m/s if the message is from the jaws
    velocity = packet.getDeviceId() == alpha::driver::DeviceId::kLinearJaws ? velocity / 1000 : velocity;

    const std::lock_guard<std::mutex> lock(access_async_states_);
    // RCLCPP_INFO(
    //     rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "async velocity is %f", velocity);

    auto deviceId = static_cast<uint8_t>(packet.getDeviceId()); // Cast the device ID to uint8_t

    auto it = std::find_if(hw_joint_structs_ref.begin(), hw_joint_structs_ref.end(),
                           [deviceId](const Joint &joint)
                           { return joint.device_id == deviceId; });

    if (it != hw_joint_structs_ref.end())
    {
      it->async_state_.velocity = velocity;
    }
  }

  void RRBotSystemMultiInterfaceHardware::updateCurrentCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_structs_ref)
  {
    if (packet.getData().size() != 4)
    {
      return;
    }

    float current;
    std::memcpy(&current, &packet.getData()[0], sizeof(current)); // NOLINT

    // Convert from mm/s to m/s if the message is from the jaws
    current = packet.getDeviceId() == alpha::driver::DeviceId::kLinearJaws ? current / 1000 : current;

    const std::lock_guard<std::mutex> lock(access_async_states_);
    // RCLCPP_INFO(
    //     rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "async current is %f", current);

    auto deviceId = static_cast<uint8_t>(packet.getDeviceId()); // Cast the device ID to uint8_t

    auto it = std::find_if(hw_joint_structs_ref.begin(), hw_joint_structs_ref.end(),
                           [deviceId](const Joint &joint)
                           { return joint.device_id == deviceId; });

    if (it != hw_joint_structs_ref.end())
    {
      it->async_state_.current = current;
    }
  }
  void RRBotSystemMultiInterfaceHardware::pollState(const int freq) const
  {
    while (running_.load())
    {
      driver_.request(alpha::driver::PacketId::PacketID_VELOCITY, alpha::driver::DeviceId::kLinearJaws);
      driver_.request(alpha::driver::PacketId::PacketID_VELOCITY, alpha::driver::DeviceId::kRotateEndEffector);
      driver_.request(alpha::driver::PacketId::PacketID_VELOCITY, alpha::driver::DeviceId::kBendElbow);
      driver_.request(alpha::driver::PacketId::PacketID_VELOCITY, alpha::driver::DeviceId::kBendShoulder);
      driver_.request(alpha::driver::PacketId::PacketID_VELOCITY, alpha::driver::DeviceId::kRotateBase);

      driver_.request(alpha::driver::PacketId::PacketID_POSITION, alpha::driver::DeviceId::kLinearJaws);
      driver_.request(alpha::driver::PacketId::PacketID_POSITION, alpha::driver::DeviceId::kRotateEndEffector);
      driver_.request(alpha::driver::PacketId::PacketID_POSITION, alpha::driver::DeviceId::kBendElbow);
      driver_.request(alpha::driver::PacketId::PacketID_POSITION, alpha::driver::DeviceId::kBendShoulder);
      driver_.request(alpha::driver::PacketId::PacketID_POSITION, alpha::driver::DeviceId::kRotateBase);

      driver_.request(alpha::driver::PacketId::PacketID_CURRENT, alpha::driver::DeviceId::kLinearJaws);
      driver_.request(alpha::driver::PacketId::PacketID_CURRENT, alpha::driver::DeviceId::kRotateEndEffector);
      driver_.request(alpha::driver::PacketId::PacketID_CURRENT, alpha::driver::DeviceId::kBendElbow);
      driver_.request(alpha::driver::PacketId::PacketID_CURRENT, alpha::driver::DeviceId::kBendShoulder);
      driver_.request(alpha::driver::PacketId::PacketID_CURRENT, alpha::driver::DeviceId::kRotateBase);

      std::this_thread::sleep_for(std::chrono::seconds(1 / freq));
    }
  }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::RRBotSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)

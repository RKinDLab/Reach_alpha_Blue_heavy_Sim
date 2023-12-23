// Copyright 2021 Department of Engineering Cybernetics, NTNU
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

#include "ros2_control_reach_5/reach_system_multi_Interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ros2_control_reach_5/device_id.hpp"
#include "ros2_control_reach_5/mode.hpp"
#include "ros2_control_reach_5/packet_id.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_reach_5
{
  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_init(
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

    hw_joint_structs_.reserve(info_.joints.size());
    control_modes_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      std::string device_id_value = joint.parameters.at("device_id");
      double default_position = stod(joint.parameters.at("home"));
      uint8_t device_id = static_cast<uint8_t>(std::stoul(device_id_value, nullptr, 16));

      RCLCPP_INFO(
          rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Device with id %u found", static_cast<unsigned int>(device_id));
      RCLCPP_INFO(
          rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Device default position is %f", default_position);

      hw_joint_structs_.emplace_back(joint.name, device_id, default_position);
      // RRBotSystemMultiInterface has exactly 3 state interfaces
      // and 3 command interfaces on each joint
      if (joint.command_interfaces.size() != 3)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
            "Joint '%s' has %zu command interfaces. 3 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
            joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
            joint.command_interfaces[0].name == custom_hardware_interface::HW_IF_CURRENT))
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
            "Joint '%s' has %s command interface. Expected %s, %s, %s or %s.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY, custom_hardware_interface::HW_IF_CURRENT, hardware_interface::HW_IF_ACCELERATION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 4)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
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
            rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
            "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY, custom_hardware_interface::HW_IF_CURRENT);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_configure(const rclcpp_lifecycle::State &)
  {
    // Start the driver
    try
    {
      driver_.start(cfg_.serial_port_);
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL( // NOLINT
          rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
          "Failed to configure the serial driver for the AlphaHardware system interface.");

      return hardware_interface::CallbackReturn::ERROR;
    }

    // // Register callbacks for joint states
    // driver_.subscribe(
    //     alpha::driver::PacketId::kPosition,
    //     [this](const alpha::driver::Packet &packet) -> void
    //     { updatePositionCb(packet); });

    // driver_.subscribe(
    //     alpha::driver::PacketId::kVelocity,
    //     [this](const alpha::driver::Packet &packet) -> void
    //     { updateVelocityCb(packet); });

    // // Start a thread to request state updates
    // running_.store(true);
    // state_request_worker_ = std::thread(&ReachSystemMultiInterfaceHardware::pollState, this, cfg_.state_update_freq_);

    RCLCPP_INFO( // NOLINT
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
        "Successfully configured the ReachSystemMultiInterfaceHardware system interface for serial communication!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO( // NOLINT
        rclcpp::get_logger("AlphaHardware"), "Shutting down the AlphaHardware system interface.");

    // running_.store(false);
    // state_request_worker_.join();
    // driver_.stop();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type ReachSystemMultiInterfaceHardware::prepare_command_mode_switch(
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
          hw_joint_structs_[i].velocity_command_ = 0;
          hw_joint_structs_[i].current_command_ = 0;
          control_modes_[i] = mode_level_t::MODE_DISABLE; // Revert to undefined
        }
      }
    }
    // Set the new command modes
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (control_modes_[i] != mode_level_t::MODE_DISABLE)
      {
        // Something else is using the joint! Abort!
        return hardware_interface::return_type::ERROR;
      }
      control_modes_[i] = new_modes[i];
    }
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateInterface>
  ReachSystemMultiInterfaceHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_structs_[i].position_state_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_structs_[i].velocity_state_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_joint_structs_[i].acceleration_state_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_joint_structs_[i].current_state_));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  ReachSystemMultiInterfaceHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_structs_[i].position_command_));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_structs_[i].velocity_command_));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_joint_structs_[i].current_command_));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Activating... please wait...");

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (std::isnan(hw_joint_structs_[i].position_state_))
      {
        hw_joint_structs_[i].position_state_ = 0;
      }
      if (std::isnan(hw_joint_structs_[i].velocity_state_))
      {
        hw_joint_structs_[i].velocity_state_ = 0;
      }
      if (std::isnan(hw_joint_structs_[i].current_state_))
      {
        hw_joint_structs_[i].current_state_ = 0;
      }
      if (std::isnan(hw_joint_structs_[i].acceleration_state_))
      {
        hw_joint_structs_[i].acceleration_state_ = 0;
      }
      if (std::isnan(hw_joint_structs_[i].position_command_))
      {
        hw_joint_structs_[i].position_command_ = 0;
      }
      if (std::isnan(hw_joint_structs_[i].velocity_command_))
      {
        hw_joint_structs_[i].velocity_command_ = 0;
      }
      if (std::isnan(hw_joint_structs_[i].current_command_))
      {
        hw_joint_structs_[i].current_command_ = 0;
      }
      control_modes_[i] = mode_level_t::MODE_DISABLE;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "System successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Deactivating... please wait...");

    RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type ReachSystemMultiInterfaceHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      switch (control_modes_[i])
      {
      case mode_level_t::MODE_DISABLE:
        // RCLCPP_INFO(
        //   rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        //   "Nothing is using the hardware interface!");
        return hardware_interface::return_type::OK;
        break;
      case mode_level_t::MODE_POSITION:
        hw_joint_structs_[i].acceleration_state_ = 0;
        hw_joint_structs_[i].current_state_ = 0;
        hw_joint_structs_[i].velocity_state_ = 0;
        hw_joint_structs_[i].position_state_ +=
            (hw_joint_structs_[i].position_command_ - hw_joint_structs_[i].position_state_) / 20;
        break;
      case mode_level_t::MODE_VELOCITY:

        hw_joint_structs_[i].acceleration_state_ = 0;
        hw_joint_structs_[i].current_state_ = 0;
        hw_joint_structs_[i].velocity_state_ = hw_joint_structs_[i].velocity_command_;
        hw_joint_structs_[i].position_state_ += (hw_joint_structs_[i].velocity_state_ * period.seconds()) / 20;
        break;
      case mode_level_t::MODE_CURRENT:
        hw_joint_structs_[i].current_state_ = hw_joint_structs_[i].current_command_;
        hw_joint_structs_[i].acceleration_state_ = hw_joint_structs_[i].current_command_ / 2; // dummy
        hw_joint_structs_[i].velocity_state_ += (hw_joint_structs_[i].acceleration_state_ * period.seconds()) / 20;
        hw_joint_structs_[i].position_state_ += (hw_joint_structs_[i].velocity_state_ * period.seconds()) / 20;
        break;
      case mode_level_t::MODE_STANDBY:
        hw_joint_structs_[i].current_state_ = hw_joint_structs_[i].current_command_;
        hw_joint_structs_[i].acceleration_state_ = hw_joint_structs_[i].current_command_ / 2; // dummy
        hw_joint_structs_[i].velocity_state_ += (hw_joint_structs_[i].acceleration_state_ * period.seconds()) / 20;
        hw_joint_structs_[i].position_state_ += (hw_joint_structs_[i].velocity_state_ * period.seconds()) / 20;
        break;
      }
      // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
      // RCLCPP_INFO(
      //   rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
      //   "Got pos: %.5f, vel: %.5f, acc: %.5f, cur: %.5f for joint %s!",
      //   hw_joint_structs_[i].position_state_,
      //   hw_joint_structs_[i].velocity_state_,
      //   hw_joint_structs_[i].acceleration_state_,
      //   hw_joint_structs_[i].current_state_, info_.joints[i].name.c_str());
      // END: This part here is for exemplary purposes - Please do not copy to your production code
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ReachSystemMultiInterfaceHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    // for (std::size_t i = 0; i < info_.joints.size(); i++)
    // {
    //   // Simulate sending commands to the hardware
    //   RCLCPP_INFO(
    //     rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
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

  void ReachSystemMultiInterfaceHardware::updatePositionCb(const alpha::driver::Packet &packet)
  {
    if (packet.getData().size() != 4)
    {
      return;
    }

    float position;
    std::memcpy(&position, &packet.getData()[0], sizeof(position)); // NOLINT

    // Convert from mm to m if the message is from the jaws
    position =
        packet.getDeviceId() == alpha::driver::DeviceId::kLinearJaws ? position / 1000 : position;

    const std::lock_guard<std::mutex> lock(access_async_states_);

    // We assume that the device ID is the index within the vector
    async_states_positions_[static_cast<std::size_t>(packet.getDeviceId()) - 1] = position;
  }

  void ReachSystemMultiInterfaceHardware::updateVelocityCb(const alpha::driver::Packet &packet)
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

    // We assume that the device ID is the index within the vector
    async_states_velocities_[static_cast<std::size_t>(packet.getDeviceId()) - 1] = velocity;
  }

  void ReachSystemMultiInterfaceHardware::pollState(const int freq) const
  {
    while (running_.load())
    {
      // There are a few important things to note here:
      //   1. Yes, we could use the kAllJoints device ID, but for some reason, the response rate for
      //      each joints becomes less reliable when we use kAllJoints. We get better response rates
      //      from the joints when we split this up.
      //   2. Yes, we could also create a request with multiple packet IDs, but, again, there are
      //      bugs in the serial communication when that is used. Specifically, data is more likely
      //      to become corrupted, resulting in bad reads. So instead we just request them separately.
      driver_.request(alpha::driver::PacketId::kVelocity, alpha::driver::DeviceId::kLinearJaws);
      driver_.request(alpha::driver::PacketId::kVelocity, alpha::driver::DeviceId::kRotateEndEffector);
      driver_.request(alpha::driver::PacketId::kVelocity, alpha::driver::DeviceId::kBendElbow);
      driver_.request(alpha::driver::PacketId::kVelocity, alpha::driver::DeviceId::kBendShoulder);
      driver_.request(alpha::driver::PacketId::kVelocity, alpha::driver::DeviceId::kRotateBase);

      driver_.request(alpha::driver::PacketId::kPosition, alpha::driver::DeviceId::kLinearJaws);
      driver_.request(alpha::driver::PacketId::kPosition, alpha::driver::DeviceId::kRotateEndEffector);
      driver_.request(alpha::driver::PacketId::kPosition, alpha::driver::DeviceId::kBendElbow);
      driver_.request(alpha::driver::PacketId::kPosition, alpha::driver::DeviceId::kBendShoulder);
      driver_.request(alpha::driver::PacketId::kPosition, alpha::driver::DeviceId::kRotateBase);

      std::this_thread::sleep_for(std::chrono::seconds(1 / freq));
    }
  }

} // namespace ros2_control_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_reach_5::ReachSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)

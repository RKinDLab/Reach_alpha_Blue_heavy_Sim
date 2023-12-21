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

#include "ros2_control_demo_example_3/reach_system_multi_Interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_3
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

    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = stoi(info_.hardware_parameters["baud_rate"]);

    hw_joint_structs_.reserve(info_.joints.size());
    control_level_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);

    comms_.connect(cfg_.device, cfg_.baud_rate);
    bool is_connected = comms_.connected();
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "serial is connected : %s",
        is_connected ? "true" : "false");

    int ind_j_ = 0;
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      hw_joint_structs_.emplace_back(joint.name, ind_j_++);
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

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Activating... please wait...");

    // for (int i = 0; i < cfg_.hw_start_sec_; i++)
    // {
    //   rclcpp::sleep_for(std::chrono::seconds(1));
    //   RCLCPP_INFO(
    //       rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "%.1f seconds left...",
    //       cfg_.hw_start_sec_ - i);
    // }
    // END: This part here is for exemplary purposes - Please do not copy to your production code
    // [1.92466658e-02, 1.45291960e+00 ,4.35027387e-03, 5.73423624e+00]
    // Set some default values
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
      control_level_[i] = mode_level_t::MODE_DISABLE;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "System successfully activated! %u",
        control_level_[0]);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Deactivating... please wait...");

    // for (int i = 0; i < cfg_.hw_stop_sec_; i++)
    // {
    //   rclcpp::sleep_for(std::chrono::seconds(1));
    //   RCLCPP_INFO(
    //       rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "%.1f seconds left...",
    //       cfg_.hw_stop_sec_ - i);
    // }

    RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Successfully deactivated!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type ReachSystemMultiInterfaceHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    // double reader = 0.0;
    // uint8_t device_id = 0x03;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      switch (control_level_[i])
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

        // comms_.readEncoderValues(device_id, PacketID_POSITION, reader);

        // RCLCPP_INFO(
        //     rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "%d encoder reading is : %f",
        //     device_id, reader);

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

} // namespace ros2_control_demo_example_3

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_demo_example_3::ReachSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)

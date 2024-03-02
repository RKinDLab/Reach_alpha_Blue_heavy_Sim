#include "ros2_control_blue_reach_5/vehicle_system_multi_interface.hpp"

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
  hardware_interface::CallbackReturn VehicleSystemMultiInterfaceHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    cfg_.hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    cfg_.hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    cfg_.hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);

    hw_thrust_structs_.reserve(info_.joints.size());
    control_level_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {

      Thruster::State defaultState{0.0, 0.0, 0.0, 0.0};
      hw_thrust_structs_.emplace_back(joint.name, defaultState);
      // RRBotSystemMultiInterface has exactly 3 state interfaces
      // and 3 command interfaces on each joint
      if (joint.command_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
            "Thruster '%s' has %zu command interfaces. 3 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
            joint.command_interfaces[0].name == custom_hardware_interface::HW_IF_CURRENT
            ))
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
            "Thruster '%s' has %s command interface. Expected %s, %s or %s.", joint.name.c_str(), 
            joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_VELOCITY, custom_hardware_interface::HW_IF_CURRENT , hardware_interface::HW_IF_ACCELERATION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 4)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
            "Thruster '%s'has %zu state interfaces. 3 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
            joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
            joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION ||
            joint.state_interfaces[0].name == custom_hardware_interface::HW_IF_CURRENT))
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
            "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY, custom_hardware_interface::HW_IF_CURRENT);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  VehicleSystemMultiInterfaceHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_thrust_structs_[i].current_state_.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_thrust_structs_[i].current_state_.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_thrust_structs_[i].current_state_.acceleration));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_thrust_structs_[i].current_state_.current));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  VehicleSystemMultiInterfaceHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_thrust_structs_[i].command_state_.velocity));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_thrust_structs_[i].command_state_.current));
    }

    return command_interfaces;
  }

  hardware_interface::return_type VehicleSystemMultiInterfaceHardware::prepare_command_mode_switch(
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
          hw_thrust_structs_[i].command_state_.velocity = 0;
          hw_thrust_structs_[i].command_state_.current = 0;
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

  hardware_interface::CallbackReturn VehicleSystemMultiInterfaceHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "Activating... please wait...");
        
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (std::isnan(hw_thrust_structs_[i].current_state_.position))
      {
        hw_thrust_structs_[i].current_state_.position = 0.0;
      }
      if (std::isnan(hw_thrust_structs_[i].current_state_.velocity))
      {
        hw_thrust_structs_[i].current_state_.velocity = 0.0;
      }
      if (std::isnan(hw_thrust_structs_[i].current_state_.current))
      {
        hw_thrust_structs_[i].current_state_.current = 0.0;
      }
      if (std::isnan(hw_thrust_structs_[i].current_state_.acceleration))
      {
        hw_thrust_structs_[i].current_state_.acceleration = 0.0;
      }
      if (std::isnan(hw_thrust_structs_[i].command_state_.velocity))
      {
        hw_thrust_structs_[i].command_state_.velocity = 0.0;
      }
      if (std::isnan(hw_thrust_structs_[i].command_state_.current))
      {
        hw_thrust_structs_[i].command_state_.current = 0.0;
      }
      control_level_[i] = mode_level_t::MODE_DISABLE;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "System successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn VehicleSystemMultiInterfaceHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "Deactivating... please wait...");

    for (int i = 0; i < cfg_.hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
          rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "%.1f seconds left...",
          cfg_.hw_stop_sec_ - i);
    }

    RCLCPP_INFO(rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "Successfully deactivated!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type VehicleSystemMultiInterfaceHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      switch (control_level_[i])
      {
      case mode_level_t::MODE_DISABLE:
        // RCLCPP_INFO(
        //   rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
        //   "Nothing is using the hardware interface!");
        return hardware_interface::return_type::OK;
        break;
      case mode_level_t::MODE_POSITION:
        hw_thrust_structs_[i].current_state_.current  = 0;
        hw_thrust_structs_[i].current_state_.acceleration = 0;
        hw_thrust_structs_[i].current_state_.velocity = 0;
        hw_thrust_structs_[i].current_state_.position = hw_thrust_structs_[i].command_state_.position;
        break;
      case mode_level_t::MODE_VELOCITY:
        hw_thrust_structs_[i].current_state_.acceleration = 0;
        hw_thrust_structs_[i].current_state_.current  = 0;
        hw_thrust_structs_[i].current_state_.velocity = hw_thrust_structs_[i].command_state_.velocity;
        hw_thrust_structs_[i].current_state_.position += (hw_thrust_structs_[i].current_state_.velocity * period.seconds());
        break;
      case mode_level_t::MODE_CURRENT:
        hw_thrust_structs_[i].current_state_.current  = hw_thrust_structs_[i].command_state_.current;
        hw_thrust_structs_[i].current_state_.acceleration = hw_thrust_structs_[i].command_state_.current / 2; //dummy
        hw_thrust_structs_[i].current_state_.velocity = (hw_thrust_structs_[i].current_state_.acceleration * period.seconds());
        hw_thrust_structs_[i].current_state_.position += (hw_thrust_structs_[i].current_state_.velocity * period.seconds()) / cfg_.hw_slowdown_;
        break;
      case mode_level_t::MODE_STANDBY:
        hw_thrust_structs_[i].current_state_.current  = hw_thrust_structs_[i].command_state_.current;
        hw_thrust_structs_[i].current_state_.acceleration = hw_thrust_structs_[i].command_state_.current / 2; //dummy
        hw_thrust_structs_[i].current_state_.velocity = (hw_thrust_structs_[i].current_state_.acceleration * period.seconds());
        hw_thrust_structs_[i].current_state_.position += (hw_thrust_structs_[i].current_state_.velocity * period.seconds()) / cfg_.hw_slowdown_;
        break;
      }
      // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
      // RCLCPP_INFO(
      //   rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
      //   "Got pos: %.5f, vel: %.5f, acc: %.5f, cur: %.5f for joint %s!", 
      //   hw_joint_structs_[i].position_state_,
      //   hw_joint_structs_[i].velocity_state_, 
      //   hw_joint_structs_[i].acceleration_state_,
      //   hw_joint_structs_[i].current_state_, info_.joints[i].name.c_str());
      // END: This part here is for exemplary purposes - Please do not copy to your production code
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type VehicleSystemMultiInterfaceHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    // for (std::size_t i = 0; i < info_.joints.size(); i++)
    // {
    //   // Simulate sending commands to the hardware
    //   RCLCPP_INFO(
    //     rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
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

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::VehicleSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)

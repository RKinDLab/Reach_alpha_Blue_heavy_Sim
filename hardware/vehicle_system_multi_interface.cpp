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

    blue_parameters.setupParameters();

    // Initialize the hydrodynamic parameters
    hydrodynamics_ = blue::dynamics::Vehicle(
        blue::dynamics::Inertia(blue_parameters.params.mass, blue_parameters.params.inertia_tensor_coeff, blue_parameters.params.added_mass_coeff),
        blue::dynamics::Coriolis(blue_parameters.params.mass, blue_parameters.params.inertia_tensor_coeff, blue_parameters.params.added_mass_coeff),
        blue::dynamics::Damping(blue_parameters.params.linear_damping_coeff, blue_parameters.params.quadratic_damping_coeff),
        blue::dynamics::RestoringForces(blue_parameters.params.weight, blue_parameters.params.buoyancy, blue_parameters.params.center_of_buoyancy, blue_parameters.params.center_of_gravity),
        blue::dynamics::CurrentEffects(blue_parameters.params.ocean_current));

    // Custom IOFormat for pretty printing
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]", "[", "]");
    std::stringstream ss;
    ss << hydrodynamics_.inertia.getInertia().format(CleanFmt);
    // Usage in your logging function
    RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "******************Blue vehicle inertia matrix:\n%s", ss.str().c_str());

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

    hw_sensor_states_.resize(
        info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

    control_level_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {

      Thruster::State defaultState{0.0, 0.0, 0.0, 0.0};
      hw_thrust_structs_.emplace_back(joint.name, defaultState);
      // RRBotSystemMultiInterface has exactly 3 state interfaces
      // and 3 command interfaces on each joint
      if (joint.command_interfaces.size() != 3)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
            "Thruster '%s' has %zu command interfaces. 3 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
            joint.command_interfaces[0].name == custom_hardware_interface::HW_IF_CURRENT ||
            joint.command_interfaces[0].name == "effort"))
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
            "Thruster '%s' has %s command interface. Expected %s, %s or %s.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_VELOCITY, custom_hardware_interface::HW_IF_CURRENT, hardware_interface::HW_IF_ACCELERATION);
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
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_thrust_structs_[i].current_state_.effort));
    }

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &hydrodynamics_.state[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[1].name, &hydrodynamics_.state[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[2].name, &hydrodynamics_.state[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[3].name, &hydrodynamics_.state[3]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[4].name, &hydrodynamics_.state[4]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[5].name, &hydrodynamics_.state[5]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[6].name, &hydrodynamics_.state[6]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &hydrodynamics_.state[7]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &hydrodynamics_.state[8]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[9].name, &hydrodynamics_.state[9]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[10].name, &hydrodynamics_.state[10]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[11].name, &hydrodynamics_.state[11]));
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
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_thrust_structs_[i].command_state_.effort));
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
        if (key == info_.joints[i].name + "/" + custom_hardware_interface::HW_IF_FREE_EXCITE)
        {
          new_modes.push_back(mode_level_t::MODE_FREE_EXCITE);
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT)
        {
          new_modes.push_back(mode_level_t::MODE_EFFORT);
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
        hw_thrust_structs_[i].current_state_.current = 0;
        hw_thrust_structs_[i].current_state_.acceleration = 0;
        hw_thrust_structs_[i].current_state_.velocity = 0;
        hw_thrust_structs_[i].current_state_.position = hw_thrust_structs_[i].command_state_.position;
        break;
      case mode_level_t::MODE_VELOCITY:
        hw_thrust_structs_[i].current_state_.acceleration = 0;
        hw_thrust_structs_[i].current_state_.current = 0;
        hw_thrust_structs_[i].current_state_.velocity = hw_thrust_structs_[i].command_state_.velocity;
        hw_thrust_structs_[i].current_state_.position += (hw_thrust_structs_[i].current_state_.velocity * period.seconds());
        break;
      case mode_level_t::MODE_CURRENT:
        hw_thrust_structs_[i].current_state_.current = hw_thrust_structs_[i].command_state_.current;
        hw_thrust_structs_[i].current_state_.acceleration = hw_thrust_structs_[i].command_state_.current / 2; // dummy
        hw_thrust_structs_[i].current_state_.velocity = (hw_thrust_structs_[i].current_state_.acceleration * period.seconds());
        hw_thrust_structs_[i].current_state_.position += (hw_thrust_structs_[i].current_state_.velocity * period.seconds()) / cfg_.hw_slowdown_;
        break;
      case mode_level_t::MODE_STANDBY:
        hw_thrust_structs_[i].current_state_.current = hw_thrust_structs_[i].command_state_.current;
        hw_thrust_structs_[i].current_state_.acceleration = hw_thrust_structs_[i].command_state_.current / 2; // dummy
        hw_thrust_structs_[i].current_state_.velocity = (hw_thrust_structs_[i].current_state_.acceleration * period.seconds());
        hw_thrust_structs_[i].current_state_.position += (hw_thrust_structs_[i].current_state_.velocity * period.seconds()) / cfg_.hw_slowdown_;
        break;
      case mode_level_t::MODE_EFFORT:
        // RCLCPP_INFO(
        //     rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
        //     "Got commands: %.5f,  %.5f, %.5f, %.5f, %.5f,  %.5f, %.5f, %.5f ",
        //     hw_thrust_structs_[0].command_state_.effort,
        //     hw_thrust_structs_[1].command_state_.effort,
        //     hw_thrust_structs_[2].command_state_.effort,
        //     hw_thrust_structs_[3].command_state_.effort,
        //     hw_thrust_structs_[4].command_state_.effort,
        //     hw_thrust_structs_[5].command_state_.effort,
        //     hw_thrust_structs_[6].command_state_.effort,
        //     hw_thrust_structs_[7].command_state_.effort);
        break;
      default:
        // Existing code for default case...
        break;
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type VehicleSystemMultiInterfaceHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    Eigen::Vector6d torqu;

    Eigen::VectorXd thruster_forces = Eigen::VectorXd::Map(
        (std::vector<double>{
             hw_thrust_structs_[0].command_state_.effort,
             hw_thrust_structs_[1].command_state_.effort,
             hw_thrust_structs_[2].command_state_.effort,
             hw_thrust_structs_[3].command_state_.effort,
             hw_thrust_structs_[4].command_state_.effort,
             hw_thrust_structs_[5].command_state_.effort,
             hw_thrust_structs_[6].command_state_.effort,
             hw_thrust_structs_[7].command_state_.effort})
            .data(),
        8);

    torqu = blue_parameters.params.tcm_ * thruster_forces;

    double dt = 0.001;                    // Time step for integration
    double total_time = period.seconds(); // Total time to integrate
    hydrodynamics_.tau = torqu;

    // Create a stepper; using a simple one for this example
    runge_kutta4<state_type> stepper;
    // Function to perform integration
    // We use integrate_const to integrate over a fixed time step
    integrate_const(stepper, hydrodynamics_, hydrodynamics_.state, 0.0, total_time, dt);

    return hardware_interface::return_type::OK;
  }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::VehicleSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)

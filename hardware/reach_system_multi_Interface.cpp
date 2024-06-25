#include "ros2_control_blue_reach_5/reach_system_multi_Interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <random>

#include "ros2_control_blue_reach_5/device_id.hpp"
#include "ros2_control_blue_reach_5/mode.hpp"
#include "ros2_control_blue_reach_5/packet_id.hpp"
#include "ros2_control_blue_reach_5/motor_control.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace casadi;

namespace ros2_control_blue_reach_5
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

    // std::random_device rd; // Obtain a random number from hardware
    // std::mt19937 gen(rd()); // Seed the generator
    // std::normal_distribution<> distr(0., 0.001); // Define the mean and stddev
    // std::vector<double> noise(8);

    // Print the CasADi version
    std::string casadi_version = CasadiMeta::version();
    RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "CasADi version: %s", casadi_version.c_str());
    RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Testing casadi ready for operations");
    // Use CasADi's "external" to load the compiled dynamics functions
    dynamics_service.usage_cplusplus_checks("test", "libtest.so");
    dynamics_service.forward_dynamics = dynamics_service.load_casadi_fun("Xnext", "libXnext.so");
    dynamics_service.forward_kinematics = dynamics_service.load_casadi_fun("T_fk", "libTfk.so");
    dynamics_service.kalman_filter = dynamics_service.load_casadi_fun("KF_PREDICT", "libKFnext.so");

    std::vector<double> x_est = {0.0, 0.0, 0.0};                                                         // Initial estimate of state
    double dt = 0.1;                                                                                     // time step
    double sigma_a = 0.5;                                                                                // acceleration process noise
    std::vector<double> sigma_m = {0.5, 0.5};                                                            // position measurement noise
    std::vector<double> p_0d = {5, 0, 0, 0, 5, 0, 0, 0, 10};                                             // initial covariance matrix
    std::vector<double> z = {1, 0.5};                                                                    // example measurment (position and velocity)
    std::vector<DM> arg = {DM(x_est), DM(z), reshape(DM(p_0d), 3, 3), DM(sigma_m), DM(sigma_a), DM(dt)}; // KF argument bundled
    std::vector<DM> estimates_dm = dynamics_service.kalman_filter(arg);                                  // execute KF

    std::cout << "*********************************************************" << std::endl;
    std::cout << "Kalman filter example exercise" << std::endl;
    std::cout << "Kalman filter estimate result: " << estimates_dm.at(0) << std::endl;
    std::cout << "Kalman filter error result: " << estimates_dm.at(1) << std::endl;
    std::cout << "Kalman filter Pk result: " << estimates_dm.at(2) << std::endl;
    std::cout << "*********************************************************" << std::endl;

    std::vector<double> x = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> u = {0.0, 0.0, 0.0, 0.001};
    std::vector<DM> argfd = {DM(x), DM(u)};
    std::vector<DM> resfd = dynamics_service.forward_dynamics(argfd);

    std::cout << "forward dynamics example result: " << resfd.at(0) << std::endl;
    std::cout << "*********************************************************" << std::endl;
    std::vector<double> q = {0.0, 0.0, 0.0, 0.001};
    std::vector<DM> argtk = {DM(q)};
    std::vector<DM> restk = dynamics_service.forward_kinematics(argtk);

    std::cout << "forward kinematics example result: " << restk.at(0) << std::endl;
    std::cout << "*********************************************************" << std::endl;
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Successful initialization of robot kinematics & dynamics");

    cfg_.serial_port_ = info_.hardware_parameters["serial_port"];
    cfg_.state_update_freq_ = std::stoi(info_.hardware_parameters["state_update_frequency"]);

    hw_joint_structs_.reserve(info_.joints.size());
    control_modes_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Hardware update rate is %u Hz", static_cast<int>(cfg_.state_update_freq_));

    excite = true;
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      std::string device_id_value = joint.parameters.at("device_id");
      double default_position = stod(joint.parameters.at("home"));
      uint8_t device_id = static_cast<uint8_t>(std::stoul(device_id_value, nullptr, 16));
      RCLCPP_INFO(
          rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Device with id %u found", static_cast<unsigned int>(device_id));

      double max_effort = stod(joint.parameters.at("max_effort"));
      bool positionLimitsFlag = stoi(joint.parameters.at("has_position_limits"));
      double min_position = stod(joint.parameters.at("min_position"));
      double max_position = stod(joint.parameters.at("max_position"));
      double max_velocity = stod(joint.parameters.at("max_velocity"));
      double soft_k_position = stod(joint.parameters.at("soft_k_position"));
      double soft_k_velocity = stod(joint.parameters.at("soft_k_velocity"));
      double soft_min_position = stod(joint.parameters.at("soft_min_position"));
      double soft_max_position = stod(joint.parameters.at("soft_max_position"));

      Joint::State initialState{default_position, 0.0, 0.0};
      Joint::Limits jointLimits{.position_min = min_position, .position_max = max_position, .velocity_max = max_velocity, .effort_max = max_effort};
      Joint::SoftLimits jointSoftLimits{.position_k = soft_k_position, .velocity_k = soft_k_velocity, .position_min = soft_min_position, .position_max = soft_max_position};

      hw_joint_structs_.emplace_back(joint.name, device_id, initialState, jointLimits, positionLimitsFlag, jointSoftLimits);
      // ReachSystemMultiInterface has exactly 3 state interfaces
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
      if (joint.state_interfaces.size() != 9)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
            "Joint '%s'has %zu state interfaces. 9 expected.",
            joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
            joint.state_interfaces[1].name == custom_hardware_interface::HW_IF_FILTERED_POSITION ||
            joint.state_interfaces[2].name == hardware_interface::HW_IF_VELOCITY ||
            joint.state_interfaces[3].name == custom_hardware_interface::HW_IF_FILTERED_VELOCITY ||
            joint.state_interfaces[4].name == hardware_interface::HW_IF_ACCELERATION ||
            joint.state_interfaces[5].name == custom_hardware_interface::HW_IF_ESTIMATED_ACCELERATION ||
            joint.state_interfaces[6].name == custom_hardware_interface::HW_IF_CURRENT ||
            joint.state_interfaces[7].name == hardware_interface::HW_IF_EFFORT ||
            joint.state_interfaces[8].name == custom_hardware_interface::HW_IF_STATE_ID))
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
            "Joint '%s' has %ld state interfaces. Expected %s, %s, %s, %s, %s, %s, %s , %s or %s.",
            joint.name.c_str(),
            joint.state_interfaces.size(),
            hardware_interface::HW_IF_POSITION,
            custom_hardware_interface::HW_IF_FILTERED_POSITION,
            hardware_interface::HW_IF_VELOCITY,
            custom_hardware_interface::HW_IF_FILTERED_VELOCITY,
            hardware_interface::HW_IF_ACCELERATION,
            custom_hardware_interface::HW_IF_ESTIMATED_ACCELERATION,
            custom_hardware_interface::HW_IF_CURRENT,
            hardware_interface::HW_IF_EFFORT,
            custom_hardware_interface::HW_IF_STATE_ID);
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
    state_request_worker_ = std::thread(&ReachSystemMultiInterfaceHardware::pollState, this, cfg_.state_update_freq_);

    RCLCPP_INFO( // NOLINT
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
        "Successfully configured the ReachSystemMultiInterfaceHardware system interface for serial communication!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO( // NOLINT
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Shutting down the AlphaHardware system interface.");

    running_.store(false);
    state_request_worker_.join();
    driver_.stop();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type ReachSystemMultiInterfaceHardware::prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces)
  {
    RCLCPP_INFO( // NOLINT
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "preparing command mode switch");
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
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT)
        {
          new_modes.push_back(mode_level_t::MODE_CURRENT);
        }
        if (key == info_.joints[i].name + "/" + custom_hardware_interface::HW_IF_FREE_EXCITE)
        {
          new_modes.push_back(mode_level_t::MODE_FREE_EXCITE);
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
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_structs_[i].current_state_.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_FILTERED_POSITION, &hw_joint_structs_[i].current_state_.filtered_position));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_structs_[i].current_state_.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_FILTERED_VELOCITY, &hw_joint_structs_[i].current_state_.filtered_velocity));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_joint_structs_[i].current_state_.acceleration));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ESTIMATED_ACCELERATION, &hw_joint_structs_[i].current_state_.estimated_acceleration));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_joint_structs_[i].current_state_.current));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_joint_structs_[i].current_state_.effort));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_STATE_ID, &hw_joint_structs_[i].current_state_.state_id));
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
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_structs_[i].command_state_.position));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_structs_[i].command_state_.velocity));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_joint_structs_[i].command_state_.current));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Activating... please wait...");
    try
    {
      driver_.setMode(alpha::driver::Mode::MODE_STANDBY, alpha::driver::DeviceId::kAllJoints);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), e.what()); // NOLINT
      return hardware_interface::CallbackReturn::ERROR;
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
    try
    {
      driver_.setMode(alpha::driver::Mode::MODE_DISABLE, alpha::driver::DeviceId::kAllJoints);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), e.what()); // NOLINT
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type ReachSystemMultiInterfaceHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {

    // Get access to the real-time states
    const std::lock_guard<std::mutex> lock(access_async_states_);
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      double delta_seconds = period.seconds();
      hw_joint_structs_[i].current_state_.state_id++;
      double prev_velocity_ = hw_joint_structs_[i].current_state_.filtered_velocity;
      hw_joint_structs_[i].current_state_.position = hw_joint_structs_[i].async_state_.position;                   

      hw_joint_structs_[i].current_state_.velocity = hw_joint_structs_[i].async_state_.velocity;
      hw_joint_structs_[i].current_state_.current = hw_joint_structs_[i].async_state_.current;
      hw_joint_structs_[i].current_state_.effort = motor_control.currentToTorque(i, hw_joint_structs_[i].current_state_.current);

      std::vector<double> x_est = {hw_joint_structs_[i].current_state_.filtered_position,
                                   hw_joint_structs_[i].current_state_.filtered_velocity,
                                   hw_joint_structs_[i].current_state_.estimated_acceleration};  
      double dt = delta_seconds; 
      double sigma_a = hw_joint_structs_[i].current_state_.sigma_a;        
      std::vector<double> sigma_m = hw_joint_structs_[i].current_state_.sigma_m;   
      std::vector<double> p_k = hw_joint_structs_[i].current_state_.covariance;          
      std::vector<double> yk = {hw_joint_structs_[i].current_state_.position, hw_joint_structs_[i].current_state_.velocity};
      std::vector<DM> arg = {DM(x_est), DM(yk), reshape(DM(p_k), 3, 3), DM(sigma_m), DM(sigma_a), DM(dt)};
      
      std::vector<DM> estimates_dm = dynamics_service.kalman_filter(arg);

      hw_joint_structs_[i].current_state_.filtered_position = estimates_dm.at(0).nonzeros()[0];
      hw_joint_structs_[i].current_state_.filtered_velocity = estimates_dm.at(0).nonzeros()[1];
      hw_joint_structs_[i].current_state_.estimated_acceleration = estimates_dm.at(0).nonzeros()[2];
      hw_joint_structs_[i].current_state_.KF_error = estimates_dm.at(1).nonzeros();
      hw_joint_structs_[i].current_state_.covariance = estimates_dm.at(2).nonzeros(); 
      hw_joint_structs_[i].calcAcceleration(hw_joint_structs_[i].current_state_.filtered_velocity, prev_velocity_, delta_seconds);
      //  RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), " %d acceleration %f",hw_joint_structs_[i].device_id,
      //   hw_joint_structs_[i].acceleration_state_);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ReachSystemMultiInterfaceHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    // Send the commands for each joint
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      switch (control_modes_[i])
      {
      case mode_level_t::MODE_POSITION:
        if (!std::isnan(hw_joint_structs_[i].command_state_.position))
        {
          // Get the target device
          const auto target_device = static_cast<alpha::driver::DeviceId>(hw_joint_structs_[i].device_id);

          // Get the target position; if the command is for the jaws, then convert from m to mm
          const double target_position =
              static_cast<alpha::driver::DeviceId>(hw_joint_structs_[i].device_id) == alpha::driver::DeviceId::kLinearJaws
                  ? hw_joint_structs_[i].command_state_.position * 1000
                  : hw_joint_structs_[i].command_state_.position;
          driver_.setPosition(target_position, target_device);
        }
        break;
      case mode_level_t::MODE_VELOCITY:
        if (!std::isnan(hw_joint_structs_[i].command_state_.velocity))
        {
          // Get the target device
          const auto target_device = static_cast<alpha::driver::DeviceId>(hw_joint_structs_[i].device_id);

          // Get the target velocity; if the command is for the jaws, then convert from m/s to mm/s
          const double target_velocity =
              static_cast<alpha::driver::DeviceId>(hw_joint_structs_[i].device_id) == alpha::driver::DeviceId::kLinearJaws
                  ? hw_joint_structs_[i].command_state_.velocity * 1000
                  : hw_joint_structs_[i].command_state_.velocity;

          driver_.setVelocity(target_velocity, target_device);
        }
        break;
      case mode_level_t::MODE_CURRENT:
        if (!std::isnan(hw_joint_structs_[i].command_state_.current))
        {
          // Get the target device
          const auto target_device = static_cast<alpha::driver::DeviceId>(hw_joint_structs_[i].device_id);

          // enforce soft limit;
          // const double target_current = hw_joint_structs_[i].enforce_soft_limits();

          // enforce hard limit;
          const double enforced_target_current = hw_joint_structs_[i].enforce_hard_limits(hw_joint_structs_[i].command_state_.current);
          if (enforced_target_current == 0)
          {
          };

          // if (static_cast<int>(target_device) == 5)
          // {
          //   RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "currents sent :::%f, %f ",enforced_target_current, hw_joint_structs_[i].command_state_.current);
          //   RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "state position :::%f ",hw_joint_structs_[i].async_state_.position);
          // }

          driver_.setCurrent(enforced_target_current, target_device);
        }
        break;
      case mode_level_t::MODE_FREE_EXCITE:
      {
        const auto target_device = static_cast<alpha::driver::DeviceId>(hw_joint_structs_[i].device_id);
        // Set the command interface value for this joint excitation
        const double tau_in_electric = hw_joint_structs_[i].calculateExcitationEffortForJoint();
        hw_joint_structs_[i].limits_.phase += 0.001;
        const double enforced_target_current = hw_joint_structs_[i].enforce_hard_limits(tau_in_electric);
        if (enforced_target_current == 0)
        {
        };
        // RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "currents to be sent to %li:::%f -->  %f ", i, tau_in_electric, enforced_target_current);
        if (!i == 0)
        {
          driver_.setCurrent(enforced_target_current, target_device);
        }

        break;
      }
      case mode_level_t::MODE_STANDBY:
        // Handle standby mode if needed, or just break
        break;
      case mode_level_t::MODE_DISABLE:
        // Handle disable mode if needed, or just break
        break;
      default:
        // Existing code for default case...
        break;
      }
    }
    return hardware_interface::return_type::OK;
  }

  void ReachSystemMultiInterfaceHardware::updatePositionCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_structs_ref)
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
    //     rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "async position is %f", position);

    auto deviceId = static_cast<uint8_t>(packet.getDeviceId()); // Cast the device ID to uint8_t

    auto it = std::find_if(hw_joint_structs_ref.begin(), hw_joint_structs_ref.end(),
                           [deviceId](const Joint &joint)
                           { return joint.device_id == deviceId; });

    if (it != hw_joint_structs_ref.end())
    {
      it->async_state_.position = position;
    }
  }

  void ReachSystemMultiInterfaceHardware::updateVelocityCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_structs_ref)
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
    //     rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "async velocity is %f", velocity);

    auto deviceId = static_cast<uint8_t>(packet.getDeviceId()); // Cast the device ID to uint8_t

    auto it = std::find_if(hw_joint_structs_ref.begin(), hw_joint_structs_ref.end(),
                           [deviceId](const Joint &joint)
                           { return joint.device_id == deviceId; });

    if (it != hw_joint_structs_ref.end())
    {
      it->async_state_.velocity = velocity;
    }
  }

  void ReachSystemMultiInterfaceHardware::updateCurrentCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_structs_ref)
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
    //     rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "async current is %f", current);

    auto deviceId = static_cast<uint8_t>(packet.getDeviceId()); // Cast the device ID to uint8_t

    auto it = std::find_if(hw_joint_structs_ref.begin(), hw_joint_structs_ref.end(),
                           [deviceId](const Joint &joint)
                           { return joint.device_id == deviceId; });

    if (it != hw_joint_structs_ref.end())
    {
      it->async_state_.current = current;
    }
  }

  void ReachSystemMultiInterfaceHardware::pollState(const int freq) const
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
    ros2_control_blue_reach_5::ReachSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)

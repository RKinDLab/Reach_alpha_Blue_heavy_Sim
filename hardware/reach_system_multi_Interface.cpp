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
    dynamics_service.extended_kalman_filter = dynamics_service.load_casadi_fun("EKF_J3", "libEKFnext.so");
    dynamics_service.current2torqueMap = dynamics_service.load_casadi_fun("current_to_torque_map", "libC2T.so");
    dynamics_service.torque2currentMap = dynamics_service.load_casadi_fun("torque_to_current_map", "libT2C.so");

    std::vector<double> x_q = {0.0, 0.0, 0.0, 0.0}; // Initial estimate of state
    std::vector<double> x_q_dot = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> x_q_ddot = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> x_q_dddot = {0.0, 0.0, 0.0, 0.0};
    double G = 340;
    double Ir = 0.001;
    double tau = 0.1;
    double dt = 0.0005;
    casadi::DM P0 = 10 * DM::eye(18);
    std::vector<double> sigma_m = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    std::vector<double> sigma_p = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    std::vector<double> y = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}; // example measurment (position and velocity)
    std::vector<DM> arg = {DM(x_q), DM(x_q_dot), DM(x_q_ddot), DM(x_q_dddot), DM(G), DM(Ir), DM(tau), DM(dt),
                           P0, DM(sigma_m), DM(sigma_p), DM(y)};                 // KF argument bundled
    std::vector<DM> estimates_dm = dynamics_service.extended_kalman_filter(arg); // execute KF

    std::cout << "*********************************************************" << std::endl;
    std::cout << "extended Kalman filter example exercise" << std::endl;
    std::cout << "extended Kalman filter estimate result: " << estimates_dm.at(0) << std::endl;
    // std::cout << "extended Kalman filter Pk result: " << estimates_dm.at(1) << std::endl;
    std::cout << "extended Kalman filter error result: " << estimates_dm.at(2) << std::endl;
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
      double kt = stod(joint.parameters.at("kt"));
      double I_static = stod(joint.parameters.at("I_static"));

      Joint::State initialState{default_position, 0.0, 0.0};
      Joint::Limits jointLimits{.position_min = min_position, .position_max = max_position, .velocity_max = max_velocity, .effort_max = max_effort};
      Joint::SoftLimits jointSoftLimits{.position_k = soft_k_position, .velocity_k = soft_k_velocity, .position_min = soft_min_position, .position_max = soft_max_position};
      Joint::MotorInfo actuatorProp{.kt = kt, .I_static = I_static};
      hw_joint_structs_.emplace_back(joint.name, device_id, initialState, jointLimits, positionLimitsFlag, jointSoftLimits, actuatorProp);
      // ReachSystemMultiInterface has exactly 4 state interfaces
      // and 3 command interfaces on each joint
      if (joint.command_interfaces.size() != 4)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
            "Joint '%s' has %zu command interfaces. 4 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
      //       joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
      //       joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT ||
      //       joint.command_interfaces[0].name == custom_hardware_interface::HW_IF_CURRENT))
      // {
      //   RCLCPP_FATAL(
      //       rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
      //       "Joint '%s' has %s command interface. Expected %s, %s, %s or %s.", joint.name.c_str(),
      //       joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
      //       hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT, custom_hardware_interface::HW_IF_CURRENT);
      //   return hardware_interface::CallbackReturn::ERROR;
      // }
      if (joint.state_interfaces.size() != 11)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
            "Joint '%s'has %zu state interfaces. 11 expected.",
            joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
      //       joint.state_interfaces[1].name == custom_hardware_interface::HW_IF_FILTERED_POSITION ||
      //       joint.state_interfaces[2].name == hardware_interface::HW_IF_VELOCITY ||
      //       joint.state_interfaces[3].name == custom_hardware_interface::HW_IF_FILTERED_VELOCITY ||
      //       joint.state_interfaces[4].name == hardware_interface::HW_IF_ACCELERATION ||
      //       joint.state_interfaces[5].name == custom_hardware_interface::HW_IF_ESTIMATED_ACCELERATION ||
      //       joint.state_interfaces[6].name == custom_hardware_interface::HW_IF_CURRENT ||
      //       joint.state_interfaces[7].name == hardware_interface::HW_IF_EFFORT ||
      //       joint.state_interfaces[8].name == custom_hardware_interface::HW_IF_STATE_ID))
      // {
      //   RCLCPP_FATAL(
      //       rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
      //       "Joint '%s' has %s state interfaces. Expected %s, %s, %s, %s, %s, %s, %s , %s or %s.",
      //       joint.name.c_str(),
      //       joint.state_interfaces.size(),
      //       hardware_interface::HW_IF_POSITION,
      //       custom_hardware_interface::HW_IF_FILTERED_POSITION,
      //       hardware_interface::HW_IF_VELOCITY,
      //       custom_hardware_interface::HW_IF_FILTERED_VELOCITY,
      //       hardware_interface::HW_IF_ACCELERATION,
      //       custom_hardware_interface::HW_IF_ESTIMATED_ACCELERATION,
      //       custom_hardware_interface::HW_IF_CURRENT,
      //       hardware_interface::HW_IF_EFFORT,
      //       custom_hardware_interface::HW_IF_STATE_ID);
      //   return hardware_interface::CallbackReturn::ERROR;
      // }
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
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Command Mode Switch successful");
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

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ESTIMATED_EFFORT, &hw_joint_structs_[i].current_state_.estimated_effort));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ESTIMATED_INERTIA_ZZ, &hw_joint_structs_[i].current_state_.rotor_inertia_zz));
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
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_joint_structs_[i].command_state_.effort));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
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
    double delta_seconds = period.seconds();
    // Get access to the real-time states
    const std::lock_guard<std::mutex> lock(access_async_states_);
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      // hw_joint_structs_[i].current_state_.state_id++;
      // double prev_velocity_ = hw_joint_structs_[i].current_state_.filtered_velocity;
      hw_joint_structs_[i].current_state_.position = hw_joint_structs_[i].async_state_.position;
      hw_joint_structs_[i].current_state_.velocity = hw_joint_structs_[i].async_state_.velocity;
      hw_joint_structs_[i].current_state_.current = hw_joint_structs_[i].async_state_.current;

      std::vector<DM> T2C_arg = {DM(hw_joint_structs_[i].actuator_Properties_.kt),
                                 DM(hw_joint_structs_[i].actuator_Properties_.I_static),
                                 DM(hw_joint_structs_[i].current_state_.current)};
      std::vector<DM> torque = dynamics_service.current2torqueMap(T2C_arg);
      hw_joint_structs_[i].current_state_.effort = torque.at(0).scalar();

      //  RCLCPP_DEBUG(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), " %d acceleration %f",hw_joint_structs_[i].device_id,
      //   hw_joint_structs_[i].acceleration_state_);
    }

    std::vector<double> x_q = {hw_joint_structs_[0].current_state_.filtered_position,
                               hw_joint_structs_[1].current_state_.filtered_position,
                               hw_joint_structs_[2].current_state_.filtered_position,
                               hw_joint_structs_[3].current_state_.filtered_position}; // Initial estimate of state
    std::vector<double> x_q_dot = {hw_joint_structs_[0].current_state_.filtered_velocity,
                                   hw_joint_structs_[1].current_state_.filtered_velocity,
                                   hw_joint_structs_[2].current_state_.filtered_velocity,
                                   hw_joint_structs_[3].current_state_.filtered_velocity};
    std::vector<double> x_q_ddot = {hw_joint_structs_[0].current_state_.estimated_acceleration,
                                    hw_joint_structs_[1].current_state_.estimated_acceleration,
                                    hw_joint_structs_[2].current_state_.estimated_acceleration,
                                    hw_joint_structs_[3].current_state_.estimated_acceleration};
    std::vector<double> x_q_dddot = {hw_joint_structs_[0].current_state_.estimated_jerk,
                                     hw_joint_structs_[1].current_state_.estimated_jerk,
                                     hw_joint_structs_[2].current_state_.estimated_jerk,
                                     hw_joint_structs_[3].current_state_.estimated_jerk};
    double G = hw_joint_structs_[3].gear_ratio;
    double Ir = hw_joint_structs_[3].current_state_.rotor_inertia_zz;
    double tau = hw_joint_structs_[3].current_state_.estimated_effort;
    double dt = delta_seconds;
    casadi::DM P0 = hw_joint_structs_[3].current_state_.covariance;
    std::vector<double> sigma_m = hw_joint_structs_[3].current_state_.sigma_m;
    std::vector<double> sigma_p = hw_joint_structs_[3].current_state_.sigma_p;
    std::vector<double> y = {hw_joint_structs_[0].current_state_.position,
                               hw_joint_structs_[1].current_state_.position,
                               hw_joint_structs_[2].current_state_.position,
                               hw_joint_structs_[3].current_state_.position,
                                hw_joint_structs_[0].current_state_.velocity,
                               hw_joint_structs_[1].current_state_.velocity,
                               hw_joint_structs_[2].current_state_.velocity,
                               hw_joint_structs_[3].current_state_.velocity,
                               hw_joint_structs_[3].current_state_.effort}; // example measurment (position and velocity)
    std::vector<DM> ekf_arg = {DM(x_q), DM(x_q_dot), DM(x_q_ddot), DM(x_q_dddot), DM(G), DM(Ir), DM(tau), DM(dt),
                           P0, DM(sigma_m), DM(sigma_p), DM(y)};                 // KF argument bundled
    std::vector<DM> estimates_dm = dynamics_service.extended_kalman_filter(ekf_arg); // execute KF

    hw_joint_structs_[0].current_state_.filtered_position = estimates_dm.at(0).nonzeros()[0];
    hw_joint_structs_[1].current_state_.filtered_position = estimates_dm.at(0).nonzeros()[1];
    hw_joint_structs_[2].current_state_.filtered_position = estimates_dm.at(0).nonzeros()[2];
    hw_joint_structs_[3].current_state_.filtered_position = estimates_dm.at(0).nonzeros()[3];

    hw_joint_structs_[0].current_state_.filtered_velocity = estimates_dm.at(0).nonzeros()[4];
    hw_joint_structs_[1].current_state_.filtered_velocity = estimates_dm.at(0).nonzeros()[5];
    hw_joint_structs_[2].current_state_.filtered_velocity = estimates_dm.at(0).nonzeros()[6];
    hw_joint_structs_[3].current_state_.filtered_velocity = estimates_dm.at(0).nonzeros()[7];

    hw_joint_structs_[0].current_state_.estimated_acceleration = estimates_dm.at(0).nonzeros()[8];
    hw_joint_structs_[1].current_state_.estimated_acceleration = estimates_dm.at(0).nonzeros()[9];
    hw_joint_structs_[2].current_state_.estimated_acceleration = estimates_dm.at(0).nonzeros()[10];
    hw_joint_structs_[3].current_state_.estimated_acceleration = estimates_dm.at(0).nonzeros()[11];

    hw_joint_structs_[3].current_state_.rotor_inertia_zz = estimates_dm.at(0).nonzeros()[12];

    hw_joint_structs_[0].current_state_.estimated_jerk = estimates_dm.at(0).nonzeros()[13];
    hw_joint_structs_[1].current_state_.estimated_jerk = estimates_dm.at(0).nonzeros()[14];
    hw_joint_structs_[2].current_state_.estimated_jerk = estimates_dm.at(0).nonzeros()[15];
    hw_joint_structs_[3].current_state_.estimated_jerk = estimates_dm.at(0).nonzeros()[16];

    hw_joint_structs_[3].current_state_.estimated_effort = estimates_dm.at(0).nonzeros()[17];

    hw_joint_structs_[3].current_state_.covariance = estimates_dm.at(1);
    // hw_joint_structs_[3].current_state_.KF_error = estimates_dm.at(2).nonzeros();

    // hw_joint_structs_[i].calcAcceleration(hw_joint_structs_[i].current_state_.filtered_velocity, prev_velocity_, delta_seconds);

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

          // enforce hard limit;
          const double enforced_target_current = hw_joint_structs_[i].enforce_hard_limits(hw_joint_structs_[i].command_state_.current);

          // if (static_cast<int>(target_device) == 3)
          // {
          //   RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "currents sent :::%f ", hw_joint_structs_[i].command_state_.current);
          //   RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "state position :::%f ",hw_joint_structs_[i].async_state_.position);
          // }

          driver_.setCurrent(enforced_target_current, target_device);
          if (enforced_target_current == 0.0)
          {
            driver_.setVelocity(0.0, target_device); // incase of currents leak
          };
        }
        break;
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

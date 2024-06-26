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

#ifndef ROS2_CONTROL_BLUE_REACH_5__RRBOT_SYSTEM_MULTI_INTERFACE_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__RRBOT_SYSTEM_MULTI_INTERFACE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_blue_reach_5/visibility_control.h"

#include "ros2_control_blue_reach_5/driver.hpp"
#include "ros2_control_blue_reach_5/packet.hpp"
#include "ros2_control_blue_reach_5/joint.hpp"
#include "ros2_control_blue_reach_5/custom_hardware_interface_type_values.hpp"
#include "ros2_control_blue_reach_5/dynamics.hpp"

#include <casadi/casadi.hpp>

namespace ros2_control_blue_reach_5
{
  class RRBotSystemMultiInterfaceHardware : public hardware_interface::SystemInterface
  {

    struct Config
    {
      // Parameters for the RRBot simulation
      std::string serial_port_;
      int state_update_freq_;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemMultiInterfaceHardware);

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces) override;

    // ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    // hardware_interface::return_type perform_command_mode_switch(
    //     const std::vector<std::string> &start_interfaces,
    //     const std::vector<std::string> &stop_interfaces) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    Config cfg_;

    // Enum defining at which control level we are
    // maintaining the command_interface type per joint.
    enum mode_level_t : std::uint8_t
    {
      MODE_STANDBY = 0x00,
      MODE_DISABLE = 0x01,
      MODE_POSITION = 0x02,
      MODE_VELOCITY = 0x03,
      MODE_CURRENT = 0x04,
      MODE_EFFORT = 0x09,
    };

    // Active control mode for each actuator
    std::vector<mode_level_t> control_level_;

    // Store the state & commands for the robot joints
    std::vector<Joint> hw_joint_structs_;

    // Store the dynamics function for the robot joints
    casadi_reach_alpha_5::Dynamics dynamics_service;

    // stores the dynamic response from the forward dynamics simulator
    std::vector<double> forward_dynamics_res;

    /**
     * @brief Write the current position of the robot received from the serial client to the
     * respective asynchronous vector.
     *
     * @param packet The position packet that signaled the callback.
     */
    void updatePositionCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_structs_ref);

    /**
     * @brief Write the current velocity of the robot received from the serial client to the
     * respective asynchronous vector.
     *
     * @param packet The velocity packet that signaled the callback.
     */
    void updateVelocityCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_structs_ref);

    /**
     * @brief Asynchronously read the current state of the robot by polling the robot serial
     * interface.
     *
     * @param freq The frequency (Hz) that the interface should poll the current robot state at.
     */

    void updateCurrentCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_structs_ref);

    /**
     * @brief Asynchronously read the current state of the robot by polling the robot serial
     * interface.
     *
     * @param freq The frequency (Hz) that the interface should poll the current robot state at.
     */

    void pollState(int freq) const;

    // Driver things
    alpha::driver::Driver driver_;
    std::thread state_request_worker_;
    std::atomic<bool> running_{false};

    std::mutex access_async_states_;
  };

} // namespace ros2_control_blue_reach_5
#endif // ROS2_CONTROL_BLUE_REACH_5__RRBOT_SYSTEM_MULTI_INTERFACE_HPP_

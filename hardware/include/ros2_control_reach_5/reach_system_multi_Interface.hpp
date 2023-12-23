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

#ifndef ROS2_CONTROL_REACH_5__REACH_SYSTEM_MULTI_INTERFACE_HPP_
#define ROS2_CONTROL_REACH_5__REACH_SYSTEM_MULTI_INTERFACE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>

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
#include "ros2_control_reach_5/visibility_control.h"

#include "ros2_control_reach_5/driver.hpp"
#include "ros2_control_reach_5/packet.hpp"
#include "ros2_control_reach_5/joint.hpp"
#include "ros2_control_reach_5/custom_hardware_interface_type_values.hpp"

namespace ros2_control_reach_5
{
  class ReachSystemMultiInterfaceHardware : public hardware_interface::SystemInterface
  {

    struct Config
    {
      // Parameters for the RRBot simulation
      std::string serial_port_;
      int state_update_freq_;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ReachSystemMultiInterfaceHardware);

    ROS2_CONTROL_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    ROS2_CONTROL_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_REACH_5_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces) override;

    // ROS2_CONTROL_REACH_5_PUBLIC
    // hardware_interface::return_type perform_command_mode_switch(
    //     const std::vector<std::string> &start_interfaces,
    //     const std::vector<std::string> &stop_interfaces) override;

    ROS2_CONTROL_REACH_5_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    ROS2_CONTROL_REACH_5_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    ROS2_CONTROL_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_REACH_5_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    ROS2_CONTROL_REACH_5_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // ReachComms comms_;
    Config cfg_;

    // Enum defining at which control level we are
    // maintaining the command_interface type per joint.
    enum class mode_level_t
    {
      MODE_STANDBY,
      MODE_DISABLE,
      MODE_POSITION,
      MODE_VELOCITY,
      MODE_CURRENT,
    };
    

    // Active control mode for each actuator
    std::vector<mode_level_t> control_modes_;

    // Store the state & commands for the robot joints
    std::vector<Joint> hw_joint_structs_;

    /**
     * @brief Write the current position of the robot received from the serial client to the
     * respective asynchronous vector.
     *
     * @param packet The position packet that signaled the callback.
     */
    void updatePositionCb(const alpha::driver::Packet &packet);

    /**
     * @brief Write the current velocity of the robot received from the serial client to the
     * respective asynchronous vector.
     *
     * @param packet The velocity packet that signaled the callback.
     */
    void updateVelocityCb(const alpha::driver::Packet &packet);

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

    // ros2_control command interfaces
    std::vector<double> hw_commands_velocities_;
    std::vector<double> hw_commands_positions_;

    // ros2_control state interfaces
    std::vector<double> hw_states_positions_, async_states_positions_;
    std::vector<double> hw_states_velocities_, async_states_velocities_;

    std::mutex access_async_states_;
  };

} // namespace ros2_control_reach_5
#endif // ROS2_CONTROL_REACH_5__REACH_SYSTEM_MULTI_INTERFACE_HPP_

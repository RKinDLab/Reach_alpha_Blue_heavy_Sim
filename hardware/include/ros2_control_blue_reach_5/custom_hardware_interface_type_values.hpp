// Copyright 2023 edward morgan
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

#ifndef ROS2_CONTROL_BLUE_REACH_5__CUSTOM_HARDWARE_INTERFACE_TYPE_VALUES_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__CUSTOM_HARDWARE_INTERFACE_TYPE_VALUES_HPP_

namespace custom_hardware_interface
{
/// Constant defining current interface
constexpr char HW_IF_CURRENT[] = "current";
/// Constant defining disable interface
constexpr char HW_IF_DISABLE[] = "disable";
/// Constant defining standby interface
constexpr char HW_IF_STANDBY[] = "standby";
/// Constant defining state counts for synchronizations purposes
constexpr char HW_IF_STATE_ID[] = "stateId";
/// Constant defining filtered_position interface
constexpr char HW_IF_FILTERED_POSITION[] = "filtered_position";
/// Constant defining filtered_velocity interface
constexpr char HW_IF_FILTERED_VELOCITY[] = "filtered_velocity";
/// Constant defining estimated_acceleration interface
constexpr char HW_IF_ESTIMATED_ACCELERATION[] = "estimated_acceleration";
/// Constant defining estimated_torque interface
constexpr char HW_IF_ESTIMATED_EFFORT[] = "estimated_effort";
/// Constant defining estimated_inertia_zz interface
constexpr char HW_IF_ESTIMATED_INERTIA_ZZ[] = "estimated_inertia_zz";
}  // namespace hardware_interface

#endif  // ROS2_CONTROL_BLUE_REACH_5__CUSTOM_HARDWARE_INTERFACE_TYPE_VALUES_HPP_

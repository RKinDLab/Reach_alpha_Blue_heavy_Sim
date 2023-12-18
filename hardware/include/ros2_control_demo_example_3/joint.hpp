#ifndef ROS2_CONTROL_DEMO_EXAMPLE_3__JOINT_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_3__JOINT_HPP_

#include <string>

class Joint
{

public:
    std::string name;
    uint8_t device_id;
    double position_command_;
    double velocity_command_;
    double current_command_;
    double position_state_;
    double velocity_state_;
    double current_state_;
    double acceleration_state_;

    Joint() = default;

    // constructor with member initializer list and move semantics for string
    Joint(std::string joint_name, int joint_id)
        : name(std::move(joint_name)), device_id(joint_id) {}

    void calcAcceleration(const double &prev_velocity_, const double &period_seconds);

};

#endif // ROS2_CONTROL_DEMO_EXAMPLE_3__JOINT_HPP_
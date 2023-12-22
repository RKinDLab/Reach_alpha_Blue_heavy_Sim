#ifndef ROS2_CONTROL_REACH_5__JOINT_HPP_
#define ROS2_CONTROL_REACH_5__JOINT_HPP_

#include <string>

class Joint
{

public:
    std::string name;
    uint8_t device_id;
    double default_position_ = 0;
    double position_command_ = 0;
    double velocity_command_ = 0;
    double current_command_ = 0;
    double position_state_ = 0;
    double velocity_state_ = 0;
    double current_state_ = 0;
    double acceleration_state_ = 0;

    Joint() = default;

    // constructor with member initializer list and move semantics for string
    Joint(std::string joint_name, int joint_id, double joint_default)
        : name(std::move(joint_name)), device_id(joint_id), default_position_(joint_default) {}

    void calcAcceleration(const double &prev_velocity_, const double &period_seconds);

    void setSimHome();

};

#endif // ROS2_CONTROL_REACH_5__JOINT_HPP_
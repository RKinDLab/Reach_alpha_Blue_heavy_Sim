#ifndef ROS2_CONTROL_BLUE_REACH_5__THRUSTER_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__THRUSTER_HPP_

#include <string>
#include <algorithm>

class Thruster
{

public:
    std::string name;  // Name of the device or component

    struct State
    {
        double position = 0;
        double velocity = 0;
        double current = 0;
        double acceleration = 0;
        double effort = 0;
    };


    struct Command_State
    {
        double current = 0;
        double effort = 0;
    };

    State default_state_{}, current_state_{}, async_state_{};
    Command_State command_state_{};


    Thruster() = default;
    // Constructor with member initializer list
    Thruster(std::string joint_name, State default_state)
        : name(std::move(joint_name)),
          default_state_(default_state){}

    void calcAcceleration(const double &prev_velocity_, const double &period_seconds);

};

#endif // ROS2_CONTROL_BLUE_REACH_5__THRUSTER_HPP_
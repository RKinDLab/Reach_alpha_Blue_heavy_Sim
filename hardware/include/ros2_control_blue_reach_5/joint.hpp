#ifndef ROS2_CONTROL_BLUE_REACH_5__JOINT_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__JOINT_HPP_

#include <string>
#include <algorithm>
#include <cmath> 

class Joint
{

private:
    double soft_min_velocity = 0;
    double soft_max_velocity = 0;

public:
    std::string name;  // Name of the device or component
    uint8_t device_id; // Unique identifier for the device

    struct State
    {
        double position = 0;
        double velocity = 0;
        double current = 0;
        double acceleration = 0;
        double effort = 0;
        double state_id = 0;
    };

    State default_state_{}, command_state_{}, current_state_{}, async_state_{};

    // Device capabilities and constraints
    struct Limits
    {
        double position_min = 0;
        double position_max = 0;
        double velocity_max = 0;
        double effort_max = 0;
        double phase = 0;
    };

    Limits limits_{};
    bool has_position_limits = false;

    struct SoftLimits
    {
        double position_k = 0;
        double velocity_k = 0;
        double position_min = 0;
        double position_max = 0;
    };

    SoftLimits soft_limits_{};

    Joint() = default;
    // Constructor with member initializer list
    Joint(std::string joint_name, uint8_t joint_id, State default_state)
        : name(std::move(joint_name)),
          device_id(joint_id),
          default_state_(default_state){}

    // Constructor with member initializer list
    Joint(std::string joint_name, uint8_t joint_id, State default_state, Limits limits, bool position_limits, SoftLimits soft_limits)
        : name(std::move(joint_name)),
          device_id(joint_id),
          default_state_(default_state),
          limits_(limits),
          has_position_limits(position_limits),
          soft_limits_(soft_limits) {}

    void calcAcceleration(const double &prev_velocity_, const double &period_seconds);

    /**
     * @brief Enforce position, velocity, and effort limits for a joint that is not subject to soft limits.
     *
     * @param current_effort The serial port that the manipulator is available at.
     * is considered timed out. This must be greater than 1 second; defaults to 3 seconds.
     */
    double enforce_hard_limits(const double &current_effort);

    /**
     * @brief Enforce position, velocity and effort limits for a joint subject to soft limits.
     * @note If the joint has no position limits (eg. a continuous joint), only velocity and effort limits
     * will be enforced.
     */
    double enforce_soft_limits();

    double calculateExcitationEffortForJoint();
};

#endif // ROS2_CONTROL_BLUE_REACH_5__JOINT_HPP_
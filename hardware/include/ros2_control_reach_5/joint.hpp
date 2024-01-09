#ifndef ROS2_CONTROL_REACH_5__JOINT_HPP_
#define ROS2_CONTROL_REACH_5__JOINT_HPP_

#include <string>
#include <algorithm>

class Joint
{

public:
    std::string name;  // Name of the device or component
    uint8_t device_id; // Unique identifier for the device

    // Internal state variables
    double default_position_ = 0;     // Default position of the device
    double position_command_ = 0;     // Commanded position for the device
    double velocity_command_ = 0;     // Commanded velocity for the device
    double current_command_ = 0;      // Commanded current for the device
    double position_state_ = 0;       // Current position state of the device
    double velocity_state_ = 0;       // Current velocity state of the device
    double async_position_state_ = 0; // Asynchronous position state of the device
    double async_velocity_state_ = 0; // Asynchronous velocity state of the device
    double async_current_state_ = 0;  // Asynchronous current state of the device
    double current_state_ = 0;        // Current state of the electrical current in the device
    double acceleration_state_ = 0;   // Current acceleration state of the device

    // Device capabilities and constraints
    double max_effort = 0;               // Maximum effort or force the device can exert
    bool has_position_limits = false;      // Flag to indicate if the device has position limits (0 or 1)
    double min_position = 0;             // Minimum allowed position for the device
    double max_position = 0;             // Maximum allowed position for the device
    double max_velocity = 0;             // Maximum velocity the device can achieve
    double soft_k_position = 0;   // Position factor for soft limits calculation
    double soft_k_velocity = 0;   // Velocity factor for soft limits calculation
    double soft_min_position = 0; // Minimum position for soft limits
    double soft_max_position = 0; // Maximum position for soft limits
    double soft_min_velocity = 0;             // Minimum soft limit for velocity
    double soft_max_velocity = 0;             // Maximum soft limit for velocity

    Joint() = default;

    // constructor with member initializer list and move semantics for string
    Joint(std::string joint_name, int joint_id, double joint_default)
        : name(std::move(joint_name)),
         device_id(joint_id),
         default_position_(joint_default) {}

    void calcAcceleration(const double &prev_velocity_, const double &period_seconds);

    /**
     * @brief Enforce position, velocity, and effort limits for a joint that is not subject to soft limits.
     *
     * @param serial_port The serial port that the manipulator is available at.
     * @param heartbeat_timeout The maximum time (s) between heartbeat messages before the connection
     * is considered timed out. This must be greater than 1 second; defaults to 3 seconds.
     */
    double enforce_hard_limits();

    /**
     * @brief Enforce position, velocity and effort limits for a joint subject to soft limits.
     * @note If the joint has no position limits (eg. a continuous joint), only velocity and effort limits
     * will be enforced.
     *
     */
    double enforce_soft_limits();
};

#endif // ROS2_CONTROL_REACH_5__JOINT_HPP_
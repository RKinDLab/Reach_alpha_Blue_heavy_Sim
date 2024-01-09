#include "ros2_control_reach_5/joint.hpp"

void Joint::calcAcceleration(const double &prev_velocity_, const double &period_seconds)
{
    acceleration_state_ = (velocity_state_ - prev_velocity_) / period_seconds;
}

double Joint::enforce_hard_limits()
{
    double min_eff = -max_effort;
    double max_eff = max_effort;

    if (has_position_limits)
    {

        if (position_state_ < min_position)
        {
            min_eff = 0.0;
        }
        else if (position_state_ > max_position)
        {
            max_eff = 0.0;
        }
    }
    if (velocity_state_ < -max_velocity)
    {
        min_eff = 0.0;
    }
    else if (velocity_state_ > max_velocity)
    {
        max_eff = 0.0;
    }

    double clamped = std::clamp(current_command_, min_eff, max_eff);
    return clamped;
};

double Joint::enforce_soft_limits()
{
    if (has_position_limits)
    {
        // Velocity bounds depend on the velocity limit and the proximity to the position limit
        soft_min_velocity = std::clamp(
            -soft_k_position * (position_state_ - soft_min_position), -max_velocity,
            max_velocity);

        soft_max_velocity = std::clamp(
            -soft_k_position * (position_state_ - soft_max_position), -max_velocity,
            max_velocity);
    }
    else
    {
        // No position limits, eg. continuous joints
        soft_min_velocity = -max_velocity;
        soft_max_velocity = max_velocity;
    }

    // Effort bounds depend on the velocity and effort bounds
    const double soft_min_eff = std::clamp(
        -soft_k_velocity * (velocity_state_ - soft_min_velocity), -max_effort, max_effort);

    const double soft_max_eff = std::clamp(
        -soft_k_velocity * (velocity_state_ - soft_max_velocity), -max_effort, max_effort);

    // Saturate effort command according to bounds
    const double eff_cmd = std::clamp(current_command_, soft_min_eff, soft_max_eff);
    return eff_cmd;
};
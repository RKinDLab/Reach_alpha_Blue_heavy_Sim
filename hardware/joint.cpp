#include "ros2_control_blue_reach_5/joint.hpp"

double Joint::calculateExcitationEffortForJoint() {
    // Define a damping zone near the limits where the effort will start to decrease
    double amplitude = limits_.effort_max;
    double effort = 0.0;

    double dist_to_min = current_state_.position - limits_.position_min;
    double dist_to_max = limits_.position_max - current_state_.position;

    // Apply the sinusoidal function with the adjusted amplitude
    effort = amplitude * std::sin(limits_.phase);

    if (dist_to_min < 0.1 && effort < 0) {
        // effort = std::abs(effort); // This makes the effort positive.
        limits_.phase = -limits_.phase;
    }

    if (dist_to_max < 0.1 && effort > 0) {
        limits_.phase = -limits_.phase;
    }

    // Return the calculated effort
    return effort;
}


double Joint::enforce_hard_limits(const double &current_effort)
{
    double min_eff = -limits_.effort_max;
    double max_eff = limits_.effort_max;

    if (has_position_limits)
    {

        if (current_state_.position < limits_.position_min)
        {
            min_eff = 0.0;
        }
        else if (current_state_.position > limits_.position_max)
        {
            max_eff = 0.0;
        }
    }
    // if (current_state_.velocity < -limits_.velocity_max)
    // {
    //     min_eff = 0.0;
    // }
    // else if (current_state_.velocity > limits_.velocity_max)
    // {
    //     max_eff = 0.0;
    // }

    double clamped = std::clamp(current_effort, min_eff, max_eff);
    return clamped;
};


void Joint::calcAcceleration(const double &prev_velocity_, const double &period_seconds)
{
    current_state_.acceleration = (current_state_.velocity - prev_velocity_) / period_seconds;
}


double Joint::enforce_soft_limits()
{
    if (has_position_limits)
    {
        // Velocity bounds depend on the velocity limit and the proximity to the position limit
        soft_min_velocity = std::clamp(
            -soft_limits_.position_k * (current_state_.position - soft_limits_.position_min), -limits_.velocity_max,
            limits_.velocity_max);

        soft_max_velocity = std::clamp(
            -soft_limits_.position_k * (current_state_.position - soft_limits_.position_max), -limits_.velocity_max,
            limits_.velocity_max);
    }
    else
    {
        // No position limits, eg. continuous joints
        soft_min_velocity = -limits_.velocity_max;
        soft_max_velocity = limits_.velocity_max;
    }

    // Effort bounds depend on the velocity and effort bounds
    const double soft_min_eff = std::clamp(
        -soft_limits_.velocity_k * (current_state_.velocity - soft_min_velocity), -limits_.effort_max, limits_.effort_max);

    const double soft_max_eff = std::clamp(
        -soft_limits_.velocity_k * (current_state_.velocity - soft_max_velocity), -limits_.effort_max, limits_.effort_max);

    // Saturate effort command according to bounds
    const double eff_cmd = std::clamp(command_state_.current, soft_min_eff, soft_max_eff);
    return eff_cmd;
};
#include "ros2_control_demo_example_3/joint.hpp"

void Joint::calcAcceleration(const double &prev_velocity_, const double &period_seconds)
{
    acceleration_state_ = (velocity_state_ - prev_velocity_) / period_seconds;
}
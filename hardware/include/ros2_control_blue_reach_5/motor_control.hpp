#ifndef ROS2_CONTROL_BLUE_REACH_5__MOTOR_CONTROL_HPP
#define ROS2_CONTROL_BLUE_REACH_5__MOTOR_CONTROL_HPP

#include <array>

class MotorControl {
public:
    MotorControl(); // Constructor
    double torqueToCurrentMap(int joint, double torque);
    double currentErrorToTorqueMap(int joint, double currentExpected, double currentMeasured);
    double currentToTorque(int joint, double current);

private:
    std::array<double, 5> kt; // Torque to current coefficient
    std::array<double, 5> I_static; // Static friction current
    std::array<double, 5> t_static; // Static friction torque
};

#endif // ROS2_CONTROL_BLUE_REACH_5__MOTORCONTROL_HPP

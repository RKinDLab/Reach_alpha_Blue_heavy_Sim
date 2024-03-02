#include "ros2_control_blue_reach_5/motor_control.hpp"
#include <cmath> // for std::abs

// Constructor implementation
MotorControl::MotorControl() : kt({50, 50, 90.6, 90.6, 90.6}), I_static({43.0, 43.0, 43.0, 43.0, 43.0}) {
    for (size_t i = 0; i < kt.size(); ++i) {
        t_static[i] = I_static[i] / kt[i];
    }
}

// torqueToCurrentMap implementation
double MotorControl::torqueToCurrentMap(int joint, double torque) {
    double deadband;
    if (-t_static[joint] <= torque && torque <= t_static[joint]) {
        deadband = 0.0;
    } else if (torque >= 0) {
        deadband = I_static[joint];
    } else {
        deadband = -I_static[joint];
    }
    return kt[joint] * torque + deadband;
}

// currentErrorToTorqueMap implementation
double MotorControl::currentErrorToTorqueMap(int joint, double currentExpected, double currentMeasured) {
    double currentError = currentMeasured - currentExpected;
    double deadband;

    if (currentError >= I_static[joint]) {
        deadband = I_static[joint];
    } else if (currentError <= -I_static[joint]) {
        deadband = -I_static[joint];
    } else {
        deadband = currentError;
    }
    return (currentError - deadband) / kt[joint];
}

// currentToTorque implementation
double MotorControl::currentToTorque(int joint, double current) {
    double torque;
    if (current >= I_static[joint]) {
        torque = (current - I_static[joint]) / kt[joint];
    } else if (current <= -I_static[joint]) {
        torque = (current + I_static[joint]) / kt[joint];
    } else {
        torque = 0.0; // Within deadband, no torque
    }
    return torque;
}

#ifndef ROS2_CONTROL_BLUE_REACH_5__VEHICLE_PARAMETERS_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__VEHICLE_PARAMETERS_HPP_

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>  // Include for I/O operations

// Structure to store all vehicle parameters
struct VehicleParameters {
    double mass;
    double buoyancy;
    double weight;
    Eigen::Vector3d center_of_gravity;
    Eigen::Vector3d center_of_buoyancy;
    Eigen::VectorXd ocean_current;
    int num_thrusters;
    double control_rate;
    Eigen::Vector3d inertia_tensor_coeff;
    Eigen::VectorXd added_mass_coeff;
    Eigen::VectorXd linear_damping_coeff;
    Eigen::VectorXd quadratic_damping_coeff;
    std::vector<bool> frame;
    Eigen::MatrixXd tcm_;
};

// Class to manage the vehicle and its parameters
class Vehicle {
public:
    VehicleParameters params;  // Instance of VehicleParameters to hold all parameters

    void setupParameters();  // Method to initialize vehicle parameters
};
#endif // VEHICLE_H

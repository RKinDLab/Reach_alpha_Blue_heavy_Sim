#include "ros2_control_blue_reach_5/vehicle_parameters.hpp"

void Vehicle::setupParameters() {
    // Initialize the parameters
    params.mass = 13.5;
    params.buoyancy = 112.80;
    params.weight = 114.80;
    params.center_of_gravity = Eigen::Vector3d(0.0, 0.0, 0.0);
    params.center_of_buoyancy = Eigen::Vector3d(0.0, 0.0, 0.0);
    params.ocean_current = Eigen::VectorXd::Zero(6);
    params.num_thrusters = 8;
    params.control_rate = 200.0;
    params.inertia_tensor_coeff = Eigen::Vector3d(0.16, 0.16, 0.16);
    params.added_mass_coeff = Eigen::VectorXd::Map(
        (std::vector<double>{-5.50, -12.70, -14.60, -0.12, -0.12, -0.12}).data(), 6);
    params.linear_damping_coeff = Eigen::VectorXd::Map(
        (std::vector<double>{-4.03, -6.22, -5.18, -0.07, -0.07, -0.07}).data(), 6);
    params.quadratic_damping_coeff = Eigen::VectorXd::Map(
        (std::vector<double>{-18.18, -21.66, -36.99, -1.55, -1.55, -1.55}).data(), 6);
    params.frame = {true, true, false, false, true, false, false, true};

    // Setup the thruster configuration matrix
    std::vector<double> tcm_vec = {
        -0.707, -0.707, 0.707, 0.707, 0.0, 0.0, 0.0, 0.0,
        0.707, -0.707, 0.707, -0.707, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, -1.0, -1.0, 1.0,
        0.0, 0.0, 0.0, 0.0, -0.218, -0.218, 0.218, 0.218,
        0.0, 0.0, 0.0, 0.0, -0.12, 0.12, -0.12, 0.12,
        0.1888, -0.1888, -0.1888, 0.1888, 0.0, 0.0, 0.0, 0.0
    };
    params.tcm_ = Eigen::Map<Eigen::MatrixXd>(
        tcm_vec.data(), 6, params.num_thrusters);
    

}

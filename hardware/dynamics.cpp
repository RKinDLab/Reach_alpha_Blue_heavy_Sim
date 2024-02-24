#include "ros2_control_blue_reach_5/dynamics.hpp"

void casadi_reach_alpha_5::Dynamics::usage_cplusplus_checks(const std::string &name, const std::string &bin_name)
{
    std::cout << "---" << std::endl;
    std::cout << "Usage from CasADi C++:" << std::endl;
    std::cout << std::endl;

    // Use CasADi's "external" to load a test compiled function
    Function f = external(name, bin_name);

    // Use like any other CasADi function
    std::vector<double> x = {1, 2, 3, 4};
    std::vector<DM> arg = {reshape(DM(x), 2, 2), 5};
    std::vector<DM> res = f(arg);

    std::cout << "result (0): " << res.at(0) << std::endl;
    std::cout << "result (1): " << res.at(1) << std::endl;
}

bool casadi_reach_alpha_5::Dynamics::load_forward_dynamics(const std::string &name, const std::string &bin_name)
{
    // Use CasADi's "external" to load forward_dynamics compiled function
    forward_dynamics = external(name, bin_name);
    return true;
}

bool casadi_reach_alpha_5::Dynamics::load_forward_kinematics(const std::string &name, const std::string &bin_name)
{
    // Use CasADi's "external" to load forward_kinematics compiled function
    forward_kinematics = external(name, bin_name);
    return true;
}
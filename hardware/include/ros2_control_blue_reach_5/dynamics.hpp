#ifndef ROS2_CONTROL_BLUE_REACH_5__DYNAMICS_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__DYNAMICS_HPP_

#include <string>
#include <algorithm>
#include <casadi/casadi.hpp>

using namespace casadi;

namespace casadi_reach_alpha_5
{
    class Dynamics
    {

    public:
        uint8_t dynamics_id;       // Unique identifier for the dynamics
        Function forward_dynamics; // forward dynamics of the robotic arm

        Dynamics() = default;
        // Constructor with member initializer list
        Dynamics(uint8_t dyn_id)
            : dynamics_id(dyn_id) {}

        void usage_cplusplus_checks(const std::string &name, const std::string &bin_name);
        /**
         * @brief checks casadi function loader works well
         *
         * @param name Name as in the label assigned to a CasADi Function object
         * @param bin_name File name of the shared library
         */
        bool load_forward_dynamics(const std::string &name, const std::string &bin_name);
        /**
         * @brief checks casadi function loader works well
         *
         * @param name Name as in the label assigned to a CasADi Function object
         * @param bin_name File name of the shared library
         */
    };
} // namespace casadi_reach_alpha_5
#endif // ROS2_CONTROL_BLUE_REACH_5__JOINT_HPP_
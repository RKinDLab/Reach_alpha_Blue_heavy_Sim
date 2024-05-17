// Copyright 2023, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "ros2_control_blue_reach_5/hydrodynamics.hpp"
#include "rclcpp/rclcpp.hpp"

namespace blue::dynamics
{

  [[nodiscard]] Eigen::Matrix3d createSkewSymmetricMatrix(double a1, double a2, double a3)
  {
    Eigen::Matrix3d mat;
    mat << 0, -a3, a2, a3, 0, -a1, -a2, a1, 0;

    return mat;
  }

  [[nodiscard]] Eigen::Matrix3d createSkewSymmetricMatrix(const Eigen::Vector3d &vec)
  {
    Eigen::Matrix3d mat;
    mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;

    return mat;
  }

  Inertia::Inertia(
      double mass, const Eigen::Vector3d &inertia_tensor_coeff,
      const Eigen::Vector6d &added_mass_coeff)
  {
    Eigen::Matrix6d rigid_body = Eigen::Matrix6d::Zero();

    rigid_body.topLeftCorner(3, 3) = mass * Eigen::Matrix3d::Identity();
    rigid_body.bottomRightCorner(3, 3) = inertia_tensor_coeff.asDiagonal().toDenseMatrix();

    Eigen::Matrix6d added_mass = -added_mass_coeff.asDiagonal().toDenseMatrix();

    inertia_matrix_ = rigid_body + added_mass;
  }

  [[nodiscard]] Eigen::Matrix6d Inertia::getInertia() const { return inertia_matrix_; }

  Coriolis::Coriolis(
      double mass, const Eigen::Vector3d &inertia_tensor_coeff, Eigen::Vector6d added_mass_coeff)
      : mass_(mass),
        moments_(inertia_tensor_coeff.asDiagonal().toDenseMatrix()),
        added_mass_coeff_(std::move(added_mass_coeff))
  {
  }

  [[nodiscard]] Eigen::Matrix6d Coriolis::calculateCoriolis(const Eigen::Vector6d &velocity) const
  {
    // The rigid body Coriolis matrix only needs the angular velocity
    const Eigen::Vector3d angular_velocity = velocity.bottomRows(3);

    return calculateRigidBodyCoriolis(angular_velocity) + calculateAddedCoriolis(velocity);
  }

  [[nodiscard]] Eigen::Matrix6d Coriolis::calculateRigidBodyCoriolis(
      const Eigen::Vector3d &angular_velocity) const
  {
    Eigen::Matrix6d rigid = Eigen::Matrix6d::Zero();

    const Eigen::Vector3d moments_v2 = moments_ * angular_velocity;

    rigid.topLeftCorner(3, 3) = mass_ * createSkewSymmetricMatrix(angular_velocity);
    rigid.bottomRightCorner(3, 3) = -createSkewSymmetricMatrix(moments_v2);

    return rigid;
  }

  [[nodiscard]] Eigen::Matrix6d Coriolis::calculateAddedCoriolis(
      const Eigen::Vector6d &velocity) const
  {
    Eigen::Matrix6d added = Eigen::Matrix6d::Zero();

    Eigen::Matrix3d linear_vel = createSkewSymmetricMatrix(
        added_mass_coeff_(0) * velocity(0), added_mass_coeff_(1) * velocity(1),
        added_mass_coeff_(2) * velocity(2));

    Eigen::Matrix3d angular_vel = createSkewSymmetricMatrix(
        added_mass_coeff_(3) * velocity(3), added_mass_coeff_(4) * velocity(4),
        added_mass_coeff_(5) * velocity(5));

    added.topRightCorner(3, 3) = linear_vel;
    added.bottomLeftCorner(3, 3) = linear_vel;
    added.bottomRightCorner(3, 3) = angular_vel;

    return added;
  }

  Damping::Damping(
      const Eigen::Vector6d &linear_damping_coeff, Eigen::Vector6d quadratic_damping_coeff)
      : linear_damping_(-linear_damping_coeff.asDiagonal().toDenseMatrix()),
        quadratic_damping_coeff_(std::move(quadratic_damping_coeff))
  {
  }

  [[nodiscard]] Eigen::Matrix6d Damping::calculateDamping(const Eigen::Vector6d &velocity) const
  {
    return linear_damping_ + calculateNonlinearDamping(velocity);
  }

  [[nodiscard]] Eigen::Matrix6d Damping::calculateNonlinearDamping(
      const Eigen::Vector6d &velocity) const
  {
    return -(quadratic_damping_coeff_.asDiagonal().toDenseMatrix() * velocity.cwiseAbs())
                .asDiagonal()
                .toDenseMatrix();
  }

  RestoringForces::RestoringForces(
      double weight, double buoyancy, Eigen::Vector3d center_of_buoyancy,
      Eigen::Vector3d center_of_gravity)
      : weight_(weight),
        buoyancy_(buoyancy),
        center_of_buoyancy_(std::move(center_of_buoyancy)),
        center_of_gravity_(std::move(center_of_gravity))
  {
  }

  [[nodiscard]] Eigen::Vector6d RestoringForces::calculateRestoringForces(
      const Eigen::Matrix3d &rotation) const
  {
    const Eigen::Vector3d fg(0, 0, weight_);
    const Eigen::Vector3d fb(0, 0, -buoyancy_);

    Eigen::Vector6d g_rb;

    g_rb.topRows(3) = rotation.transpose() * (fg + fb);
    g_rb.bottomRows(3) =
        center_of_gravity_.cross(rotation.transpose() * fg) + center_of_buoyancy_.cross(rotation.transpose() * fb);

    g_rb *= -1;

    return g_rb;
  }

  CurrentEffects::CurrentEffects(Eigen::Vector6d current_velocity)
      : current_velocity_(std::move(current_velocity))
  {
  }

  [[nodiscard]] Eigen::Vector6d CurrentEffects::calculateCurrentEffects(
      const Eigen::Matrix3d &rotation) const
  {
    Eigen::Vector6d rotated_current_effects;
    rotated_current_effects.topRows(3) = rotation * current_velocity_.topRows(3);
    rotated_current_effects.bottomRows(3) = rotation * current_velocity_.bottomRows(3);

    return rotated_current_effects;
  }

  Vehicle::Vehicle(
      Inertia inertia, Coriolis coriolis, Damping damping, RestoringForces restoring_forces,
      CurrentEffects current_effects)
      : inertia(std::move(inertia)),
        coriolis(std::move(coriolis)),
        damping(std::move(damping)),
        restoring_forces(std::move(restoring_forces)),
        current_effects(std::move(current_effects))
  {
    state.resize(12);
  }

  Eigen::Matrix3d Vehicle::linear_rotation_matrix(double r, double p, double y)
  {
    // phi - roll - r
    // theta - pitch - p
    // psi - yaw - y
    Eigen::Matrix3d mat;
    double cr = cos(r), sr = sin(r);
    double cp = cos(p), sp = sin(p);
    double cy = cos(y), sy = sin(y);
    mat << cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy,
    cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy,
    -sp, sr * cp, cr * cp;
    return mat;
  }

    Eigen::Matrix3d Vehicle::angular_rotation_matrix(double r, double p)
  {
    Eigen::Matrix3d mat;
    double cr = cos(r), sr = sin(r);
    double cp = cos(p), tp = tan(p);
    mat << 1, sr*tp, cr*tp, 0, cr, -sr, 0, sr/cp, cr/cp;

    return mat;
  }

  // Function to compute acceleration
  Eigen::Vector6d Vehicle::computeAcceleration(const Eigen::Vector6d &velocity, const Eigen::Matrix3d &rot)
  {

    // // Custom IOFormat for pretty printing
    // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]", "[", "]");
    // std::stringstream ss;
    // ss << inertia.getInertia().format(CleanFmt);
    // // Usage in your logging function
    // RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "***working wow. i am azm*****Blue vehicle inertia matrix:\n%s", ss.str().c_str());

    return inertia.getInertia().inverse() *
           (tau -
            (coriolis.calculateCoriolis(velocity) + damping.calculateDamping(velocity)) * velocity 
            - restoring_forces.calculateRestoringForces(rot));
  }

  // Operator to interface with Boost Odeint
  void Vehicle::operator()(const state_type &x, state_type &dxdt, double /*t*/)
  {
    // Map the state vector to Eigen types
    Eigen::Map<const Eigen::Vector6d> vel(&x[6]);

    Eigen::Map<const Eigen::Vector3d> vb_nb(&x[6]);
    Eigen::Map<const Eigen::Vector3d> wb_nb(&x[9]);

    // std::cout << "vb_nb: " << vb_nb << std::endl;


    Eigen::Matrix3d Rot_l = linear_rotation_matrix(x[0],x[1],x[2]);
    Eigen::Matrix3d T_a = angular_rotation_matrix(x[0],x[1]);

    Eigen::Vector6d acc = computeAcceleration(vel, Rot_l);

    Eigen::Vector3d d_pn_nb  = Rot_l*vb_nb;
    Eigen::Vector3d d_thet = T_a*wb_nb;

    // std::cout << "d_pn_nb: " << d_pn_nb << std::endl;
    // std::cout << "d_thet: " << d_thet << std::endl;

  //  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]", "[", "]");
  //   std::stringstream ss;
  //   ss << R_l.format(CleanFmt);
  //   // Usage in your logging function
  //   RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "*******linear rotation matrix:\n%s", ss.str().c_str());

  //   std::stringstream sus;
  //   sus << T_a.format(CleanFmt);
  //   // Usage in your logging function
  //   RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "*****angular rotation matrix:\n%s", sus.str().c_str());

    // // Set derivatives of the position
    dxdt[0] = d_pn_nb[0]; // Linear velocity
    dxdt[1] = d_pn_nb[1]; // Linear velocity
    dxdt[2] = d_pn_nb[2]; // Linear velocity

    // set derivatives of the orientation
    dxdt[3] = d_thet[0];  // angular velocity
    dxdt[4] = d_thet[1]; // angular velocity
    dxdt[5] = d_thet[2]; // angular velocity

    dxdt[6] = acc[0]; // Linear acceleration
    dxdt[7] = acc[1]; // Linear acceleration
    dxdt[8] = acc[2]; // Linear acceleration

    dxdt[9] = acc[3];  // angular acceleration
    dxdt[10] = acc[4]; // angular acceleration
    dxdt[11] = acc[5]; // angular acceleration
  }

} // namespace blue::dynamics

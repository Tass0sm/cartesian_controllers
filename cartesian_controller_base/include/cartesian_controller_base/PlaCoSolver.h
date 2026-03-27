//-----------------------------------------------------------------------------
/*!\file    PlaCoSolver.h
 *
 * \author  Anastasios Manganaris <amangana@purdue.edu>
 * \date    2026/03/26
 *
 */
//-----------------------------------------------------------------------------

#ifndef PLACO_SOLVER_H_INCLUDED
#define PLACO_SOLVER_H_INCLUDED

#include <cartesian_controller_base/IKSolver.h>

#include <placo/model/robot_wrapper.h>
#include <placo/kinematics/kinematics_solver.h>
#include <kdl/jacobian.hpp>
#include <memory>

namespace cartesian_controller_base
{
/**
   * \brief A Jacobian transpose IK solver for Cartesian controllers
   *
   *
 *  The resulting joint accelerations are computed according to
 *  \f$ \ddot{q} = J^T f \f$
 *  Where \f$ J \f$ denotes the manipulator's joint Jacobian and \f$ f \f$ is
 *  the applied force to the end effector.
 *  This implements the dynamical system from Wolovich and Elliot
 *  https://ieeexplore.ieee.org/abstract/document/4048118 with \f$ \alpha = 1 \f$ with
 *  the difference that this implementation does not accumulate velocity during time integration.
 *  The system always starts anew in each control cycle with instantaneous
 *  accelerations, having the benefit that no damping is required to avoid overshooting.
   */
class PlaCoSolver : public IKSolver
{
public:
  PlaCoSolver();
  ~PlaCoSolver();

  /**
     * \brief Compute joint target commands with PlaCo
     *
     * \param period The duration in sec for this simulation step
     * \param net_force The applied net force, expressed in the root frame
     *
     * \return A point holding positions, velocities and accelerations of each joint
     */
  trajectory_msgs::msg::JointTrajectoryPoint getJointControlCmds(
    rclcpp::Duration period, const ctrl::Vector6D & net_force) override;

  /**
     * \brief Initialize the solver
     *
     * \param chain The kinematic chain of the robot
     * \param upper_pos_limits Tuple with max positive joint angles
     * \param lower_pos_limits Tuple with max negative joint angles
     *
     * \return True, if everything went well
     */

  bool init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> /*nh*/, const KDL::Chain & chain,
            const KDL::JntArray & upper_pos_limits,
            const KDL::JntArray & lower_pos_limits) override;

private:
  std::unique_ptr<placo::model::RobotWrapper> m_robot;
  std::unique_ptr<placo::kinematics::KinematicsSolver> m_solver;
  std::shared_ptr<KDL::ChainJntToJacSolver> m_jnt_jacobian_solver;
  KDL::Jacobian m_jnt_jacobian;
};

}  // namespace cartesian_controller_base

#endif

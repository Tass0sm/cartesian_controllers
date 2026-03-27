//-----------------------------------------------------------------------------
/*!\file    PlaCoSolver.cpp
 *
 * \author  Anastasios Manganaris <amangana@purdue.edu>
 * \date    2026/03/26
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/PlaCoSolver.h>

#include <pluginlib/class_list_macros.hpp>

/**
 * \class cartesian_controller_base::PlaCoSolver
 *
 * Users may explicitly specify this solver with \a "placo" as \a
 * ik_solver in their controllers.yaml configuration file for each controller:
 *
 * \code{.yaml}
 * <name_of_your_controller>:
 *   ros__parameters:
 *     ik_solver: "placo"
 *     ...
 * \endcode
 *
 */
PLUGINLIB_EXPORT_CLASS(cartesian_controller_base::PlaCoSolver,
                       cartesian_controller_base::IKSolver)

namespace cartesian_controller_base
{
PlaCoSolver::PlaCoSolver() {}

PlaCoSolver::~PlaCoSolver() {}

trajectory_msgs::msg::JointTrajectoryPoint PlaCoSolver::getJointControlCmds(
    rclcpp::Duration period, const ctrl::Vector6D & net_force)
{
  // Compute joint jacobian
  m_jnt_jacobian_solver->JntToJac(m_current_positions, m_jnt_jacobian);

  // Compute joint accelerations according to: \f$ \ddot{q} = H^{-1} ( J^T f) \f$
  m_current_accelerations.data = m_jnt_jacobian.data.transpose() * net_force;

  // Integrate once, starting with zero motion
  m_current_velocities.data = 0.5 * m_current_accelerations.data * period.seconds();

  // Integrate twice, starting with zero motion
  m_current_positions.data =
    m_last_positions.data + 0.5 * m_current_velocities.data * period.seconds();

  // Make sure positions stay in allowed margins
  applyJointLimits();

  // Apply results
  trajectory_msgs::msg::JointTrajectoryPoint control_cmd;
  for (int i = 0; i < m_number_joints; ++i)
  {
    control_cmd.positions.push_back(m_current_positions(i));
    control_cmd.velocities.push_back(m_current_velocities(i));

    // Accelerations should be left empty. Those values will be interpreted
    // by most hardware joint drivers as max. tolerated values. As a
    // consequence, the robot will move very slowly.
  }
  control_cmd.time_from_start = period;  // valid for this duration

  // Update for the next cycle
  m_last_positions = m_current_positions;

  return control_cmd;
}

bool PlaCoSolver::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh,
		      const KDL::Chain & chain, const KDL::JntArray & upper_pos_limits,
		      const KDL::JntArray & lower_pos_limits)
{
  IKSolver::init(nh, chain, upper_pos_limits, lower_pos_limits);

  m_jnt_jacobian_solver.reset(new KDL::ChainJntToJacSolver(m_chain));
  m_jnt_jacobian.resize(m_number_joints);

  std::string urdf_content = nh->get_parameter("robot_description").as_string();
  m_robot = std::make_unique<placo::model::RobotWrapper>(urdf_content);

  m_solver = std::make_unique<placo::kinematics::KinematicsSolver>(*m_robot);

  // 3. Define the main End-Effector Task (Position + Orientation)
  // Assuming m_end_effector_link is the name of your tip link
  m_ee_task = &m_solver->add_frame_task(m_end_effector_link, m_robot->get_frame_matrix(m_end_effector_link));
  m_ee_task->configure("ee_task", "soft", 1.0, 1.0); // Weighting for the task

  // Joint limit constraints are on by default, but manually doing this anyway.
  m_solver->enable_joint_limits(true);

  return true;
}
}  // namespace cartesian_controller_base

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
  double dt = period.seconds();

  // 1. Synchronize PlaCo's internal robot state with our current member variables
  // PlaCo expects Eigen-compatible vectors for state.q
  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(m_robot->model.nq); // Size 13
  q_full[6] = 1.0;
  q_full.tail(6) = m_current_positions.data;
  m_robot->state.q = q_full;

  Eigen::VectorXd qd_full = Eigen::VectorXd::Zero(m_robot->model.nv); // Size 12
  qd_full.tail(6) = m_current_velocities.data;
  m_robot->state.qd = qd_full;

  m_robot->update_kinematics();

  // 2. Define the Target Pose by integrating the "net_force" (interpreted as a twist)
  // We compute where we want the end-effector to be in dt seconds
  std::string ee_frame = m_robot->frame_names().back();
  Eigen::Affine3d T_current = m_robot->get_T_world_frame(ee_frame);

  // Linear integration: p_next = p_curr + v * dt
  Eigen::Vector3d next_pos = T_current.translation() + net_force.head<3>() * dt;

  // Angular integration: R_next = R_curr * exp(omega * dt)
  Eigen::Vector3d rot_vec = net_force.tail<3>() * dt;
  Eigen::Matrix3d next_rot = T_current.linear();
  if (rot_vec.norm() > 1e-9) {
    next_rot = Eigen::AngleAxisd(rot_vec.norm(), rot_vec.normalized()) * T_current.linear();
  }

  Eigen::Affine3d T_target;
  T_target.translation() = next_pos;
  T_target.linear() = next_rot;

  // 3. Update the Task in the solver
  // We assume ee_task was created during init() or retrieved here
  m_ee_task = m_solver->add_frame_task(ee_frame, T_target);
  m_ee_task.configure("ee_task", "soft", 1.0);

  // 4. Solve the QP
  // This updates m_robot->state.q to the new valid configuration
  Eigen::VectorXd qd_sol;
  try {
    qd_sol = m_solver->solve(true);
  } catch (std::runtime_error& e) {
    qd_sol.setZero(m_robot->model.nv);
    std::cerr << "PlaCo failed to find a valid solution: " << e.what() << std::endl;
  }

  // 5. Update Current State (Numerical Differentiation for Velocity)
  // We extract the new positions and calculate velocities to match your template
  m_current_positions.data = m_robot->state.q.tail(6);

  // Numerical differentiation: v = (q_new - q_old) / dt
  m_current_velocities.data = qd_sol.tail(6);

  // 6. Apply Results to message
  trajectory_msgs::msg::JointTrajectoryPoint control_cmd;
  for (int i = 0; i < m_number_joints; ++i)
  {
    control_cmd.positions.push_back(m_current_positions(i));
    control_cmd.velocities.push_back(m_current_velocities(i));
  }
  control_cmd.time_from_start = period;

  // 7. Cleanup and State Update for the next cycle
  m_solver->remove_task(m_ee_task);
  m_last_positions = m_current_positions;
  m_last_velocities = m_current_velocities;

  return control_cmd;
}

bool PlaCoSolver::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh,
		      const KDL::Chain & chain, const KDL::JntArray & upper_pos_limits,
		      const KDL::JntArray & lower_pos_limits)
{
  IKSolver::init(nh, chain, upper_pos_limits, lower_pos_limits);

  std::string robot_description = nh->get_parameter("robot_description").as_string();

  std::cout << "robot description: " << robot_description << std::endl;

  m_robot = std::make_unique<placo::model::RobotWrapper>("", 0, robot_description);
  m_solver = std::make_unique<placo::kinematics::KinematicsSolver>(*m_robot);

  std::cout << "m_robot->model.nq: " << m_robot->model.nq << std::endl;

  for (std::string s : m_robot->joint_names(false)) {
    std::cout << "joint name: " << s << std::endl;
  }

  // Joint limit constraints are on by default, but manually doing this anyway.
  m_solver->enable_joint_limits(true);

  return true;
}
}  // namespace cartesian_controller_base

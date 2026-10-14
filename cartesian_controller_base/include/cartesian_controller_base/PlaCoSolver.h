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

#include <cartesian_controller_base/AvoidObstaclesConstraint.h>
#include <cartesian_controller_base/IKSolver.h>

#include <chrono>
#include <map>
#include <memory>
#include <placo/kinematics/frame_task.h>
#include <placo/kinematics/joints_task.h>
#include <placo/kinematics/kinematics_solver.h>
#include <placo/model/robot_wrapper.h>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <string>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace cartesian_controller_base
{
/**
 * \brief A PlaCo-based IK solver for Cartesian controllers
 *
 * Each step solves a QP for the joint motion that moves the end effector by
 * the commanded twist, subject to joint limits and obstacle avoidance. The
 * URDF in \a robot_description may contain more joints than the controller's
 * chain; those (and the floating base) are held fixed.
 *
 * A low-weight posture task (\a placo.posture.*) keeps the redundant joints near
 * a nominal posture, so they don't drift into joint limits.
 *
 * Obstacle avoidance (optional): the robot's links are approximated by
 * spheres (parameters \a placo.collision_spheres.*), and obstacles are boxes
 * and spheres received as a visualization_msgs/MarkerArray on
 * \a <controller>/obstacles, in the robot base frame. The spheres are
 * published on \a <controller>/collision_spheres for checking in RViz.
 *
 * Self collisions (optional): boxes fixed to links of the robot (parameters
 * \a placo.self_collision.*, e.g. the torso) are kept clear of the same spheres,
 * like the obstacles but with their own margin. They are published with the spheres.
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
     * \param net_force The commanded twist, expressed in the robot base frame
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
  using Obstacles = std::vector<Obstacle>;  // In the robot base frame

  // A box fixed to a link of the robot, kept clear of the collision spheres
  struct BodyBox
  {
    std::string frame;
    placo::model::RobotWrapper::FrameIndex frame_index;
    Obstacle box;  // T_world_obstacle is the pose in the frame
  };

  bool initObstacleAvoidance();
  bool initSelfCollision();
  void obstaclesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  void publishCollisionSpheres();

  std::unique_ptr<placo::model::RobotWrapper> m_robot;
  std::unique_ptr<placo::kinematics::KinematicsSolver> m_solver;
  placo::kinematics::FrameTask m_ee_task;
  placo::kinematics::JointsTask * m_posture_task = nullptr;  // Owned by m_solver
  bool m_posture_from_start = false;  // No configured posture: use the one at activation
  AvoidObstaclesConstraint * m_obstacle_constraint = nullptr;  // Owned by m_solver
  std::vector<BodyBox> m_body_boxes;
  AvoidObstaclesConstraint * m_self_constraint = nullptr;  // Owned by m_solver

  std::vector<std::string> m_joint_names;  // The chain's joints, in order
  std::string m_base_frame;
  std::string m_ee_frame;

  // Obstacles by (namespace, id), maintained in the subscriber callback
  std::map<std::pair<std::string, int>, Obstacle> m_obstacle_markers;
  realtime_tools::RealtimeBuffer<Obstacles> m_obstacles_buffer;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr m_obstacles_sub;

  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> m_spheres_pub;
  std::unique_ptr<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>>
    m_spheres_rt_pub;
  std::chrono::steady_clock::time_point m_last_spheres_publish;
};

}  // namespace cartesian_controller_base

#endif

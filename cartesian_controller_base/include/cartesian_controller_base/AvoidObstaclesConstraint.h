//-----------------------------------------------------------------------------
/*!\file    AvoidObstaclesConstraint.h
 *
 * \author  Anastasios Manganaris <amangana@purdue.edu>
 * \date    2026/10/13
 *
 */
//-----------------------------------------------------------------------------

#ifndef AVOID_OBSTACLES_CONSTRAINT_H_INCLUDED
#define AVOID_OBSTACLES_CONSTRAINT_H_INCLUDED

#include <Eigen/Geometry>
#include <placo/kinematics/constraint.h>
#include <placo/model/robot_wrapper.h>
#include <string>
#include <vector>

namespace cartesian_controller_base
{
/**
 * \brief A sphere rigidly attached to a frame of the robot
 *
 * The robot's links are approximated by spheres for obstacle avoidance. This
 * keeps the distance computation cheap and works for links without collision
 * geometry in the URDF (e.g. a hand that isn't modelled).
 */
struct CollisionSphere
{
  std::string frame;
  placo::model::RobotWrapper::FrameIndex frame_index;
  Eigen::Vector3d offset;  // Center, in the frame
  double radius;
};

/**
 * \brief An obstacle in the environment
 */
struct Obstacle
{
  enum class Type
  {
    Box,
    Sphere
  };
  Type type;
  Eigen::Affine3d T_world_obstacle;  // Center and orientation
  Eigen::Vector3d half_extents;      // Box: half sizes; sphere: radius in x
};

/**
 * \brief Keeps the robot's collision spheres out of obstacles
 *
 * Velocity damper, as in PlaCo's AvoidSelfCollisionsConstraint: for every
 * sphere-obstacle pair closer than \a trigger, with distance d and unit normal
 * n (pointing from the obstacle to the sphere), the IK step dq must satisfy
 *
 *   n^T J dq >= -gain * (d - margin)
 *
 * where J is the Jacobian of the sphere's center. Each step may close at most
 * \a gain of the remaining clearance; a sphere inside the margin is pushed out
 * by that fraction of its penetration per step.
 */
class AvoidObstaclesConstraint : public placo::kinematics::Constraint
{
public:
  std::vector<CollisionSphere> spheres;
  std::vector<Obstacle> obstacles;

  double margin = 0.01;   // [m]
  double trigger = 0.05;  // [m] Pairs farther apart are ignored
  double gain = 0.1;      // Fraction of the clearance an IK step may use, in (0, 1]

  void add_constraint(placo::problem::Problem & problem) override;

  /**
   * \brief Signed distance between a sphere and an obstacle (negative when they overlap)
   *
   * \param center Sphere center in the world frame
   * \param radius Sphere radius
   * \param obstacle The obstacle
   * \param normal Unit normal in the world frame, pointing from the obstacle to the sphere
   */
  static double distance(const Eigen::Vector3d & center, double radius, const Obstacle & obstacle,
                         Eigen::Vector3d & normal);

  /**
   * \brief Centers of the collision spheres in the world frame, for the current robot state
   */
  std::vector<Eigen::Vector3d> sphereCenters() const;
};

}  // namespace cartesian_controller_base

#endif

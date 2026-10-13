//-----------------------------------------------------------------------------
/*!\file    AvoidObstaclesConstraint.cpp
 *
 * \author  Anastasios Manganaris <amangana@purdue.edu>
 * \date    2026/10/13
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/AvoidObstaclesConstraint.h>

#include <placo/kinematics/kinematics_solver.h>

namespace cartesian_controller_base
{
namespace
{
Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}
}  // namespace

double AvoidObstaclesConstraint::distance(const Eigen::Vector3d & center, double radius,
                                          const Obstacle & obstacle, Eigen::Vector3d & normal)
{
  const Eigen::Matrix3d R = obstacle.T_world_obstacle.linear();
  const Eigen::Vector3d c = obstacle.T_world_obstacle.inverse() * center;  // In the obstacle

  if (obstacle.type == Obstacle::Type::Sphere)
  {
    const double n = c.norm();
    normal = n > 1e-9 ? Eigen::Vector3d(R * (c / n)) : Eigen::Vector3d(R.col(2));
    return n - obstacle.half_extents.x() - radius;
  }

  const Eigen::Vector3d & h = obstacle.half_extents;
  const Eigen::Vector3d closest = c.cwiseMax(-h).cwiseMin(h);
  const Eigen::Vector3d outside = c - closest;
  if (outside.norm() > 1e-9)
  {
    normal = R * outside.normalized();
    return outside.norm() - radius;
  }

  // Center inside the box: leave through the nearest face
  const Eigen::Vector3d depth = h - c.cwiseAbs();
  int axis;
  depth.minCoeff(&axis);
  Eigen::Vector3d n = Eigen::Vector3d::Zero();
  n[axis] = c[axis] >= 0 ? 1.0 : -1.0;
  normal = R * n;
  return -depth[axis] - radius;
}

std::vector<Eigen::Vector3d> AvoidObstaclesConstraint::sphereCenters() const
{
  std::vector<Eigen::Vector3d> centers;
  centers.reserve(spheres.size());
  for (const auto & sphere : spheres)
  {
    centers.push_back(solver->robot.get_T_world_frame(sphere.frame_index) * sphere.offset);
  }
  return centers;
}

void AvoidObstaclesConstraint::add_constraint(placo::problem::Problem & problem)
{
  if (obstacles.empty() || spheres.empty())
  {
    return;
  }

  struct Row
  {
    size_t sphere;
    Eigen::Vector3d normal;
    double distance;
  };
  std::vector<Row> rows;
  const auto centers = sphereCenters();
  for (size_t i = 0; i < spheres.size(); ++i)
  {
    for (const auto & obstacle : obstacles)
    {
      Eigen::Vector3d normal;
      const double d = distance(centers[i], spheres[i].radius, obstacle, normal);
      if (d < trigger)
      {
        rows.push_back({i, normal, d});
      }
    }
  }
  if (rows.empty())
  {
    return;
  }

  placo::problem::Expression e;
  e.A = Eigen::MatrixXd(rows.size(), solver->N);
  e.b = Eigen::VectorXd(rows.size());
  for (size_t r = 0; r < rows.size(); ++r)
  {
    const auto & sphere = spheres[rows[r].sphere];
    // Velocity of the sphere's center: v = v_frame + w x (center - frame origin)
    const Eigen::MatrixXd J = solver->robot.frame_jacobian(sphere.frame_index);
    const Eigen::Vector3d lever = centers[rows[r].sphere] -
                                  solver->robot.get_T_world_frame(sphere.frame_index).translation();
    const Eigen::MatrixXd J_center = J.topRows(3) - skew(lever) * J.bottomRows(3);

    // We want: gain * (d - margin) + n^T J dq >= 0
    e.A.row(r) = rows[r].normal.transpose() * J_center;
    e.b[r] = gain * (rows[r].distance - margin);
  }

  problem.add_constraint(e >= 0)
    .configure(priority == Priority::Soft ? placo::problem::ProblemConstraint::Soft
                                          : placo::problem::ProblemConstraint::Hard,
               weight);
}

}  // namespace cartesian_controller_base

//-----------------------------------------------------------------------------
/*!\file    PlaCoSolver.cpp
 *
 * \author  Anastasios Manganaris <amangana@purdue.edu>
 * \date    2026/03/26
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/PlaCoSolver.h>

#include <algorithm>
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
 *     placo:
 *       posture:
 *         positions: []  # nominal posture of the chain's joints; empty: the one at activation
 *         weight: 0.00001  # relative to the end effector task (1.0); keep it small: the
 *                          # end effector task only asks for a small step each iteration, so
 *                          # a larger weight pulls the hand visibly away from its target
 *       # Optional obstacle avoidance. Spheres approximating the robot, one entry per sphere:
 *       collision_spheres:
 *         frames: ["wrist_link", "hand_link"]
 *         offsets: [0.0, 0.0, 0.0,  0.05, 0.0, 0.0]  # x y z of each center in its frame
 *         radii: [0.04, 0.05]
 *       obstacles:
 *         margin: 0.01   # [m] clearance kept from obstacles
 *         trigger: 0.05  # [m] pairs farther apart are ignored
 *         gain: 0.1      # fraction of the clearance one IK step may use, in (0, 1]
 *         hard: true     # false: soft constraint with weight `weight`
 *         weight: 1000.0
 *       # Optional self collision avoidance: boxes fixed to links of the robot, which the
 *       # collision spheres keep clear of like obstacles (same trigger, gain, hard and weight).
 *       self_collision:
 *         frames: ["torso_link"]
 *         centers: [0.0, 0.0, 0.15]  # x y z of each box center in its frame
 *         sizes: [0.15, 0.2, 0.3]    # x y z size of each box, along its frame's axes
 *         margin: 0.005  # [m] clearance kept from the boxes
 *     ...
 * \endcode
 *
 * Obstacles arrive as visualization_msgs/MarkerArray on <controller>/obstacles,
 * with header.frame_id = robot_base_link (or empty). CUBE markers are boxes
 * (scale = size), SPHERE markers spheres (scale.x = diameter). ADD replaces the
 * marker with the same (ns, id); DELETE and DELETEALL remove markers.
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
  const double dt = period.seconds();

  // Synchronize PlaCo's robot with our current joint positions
  for (int i = 0; i < m_number_joints; ++i)
  {
    m_robot->set_joint(m_joint_names[i], m_current_positions(i));
  }
  m_robot->update_kinematics();

  // The controller's activation computes one step with zero period: take that posture as nominal.
  if (m_posture_from_start && dt == 0.0)
  {
    for (int i = 0; i < m_number_joints; ++i)
    {
      m_posture_task->set_joint(m_joint_names[i], m_current_positions(i));
    }
  }

  // PlaCo's world frame is the URDF root, not necessarily the controller's base.
  const Eigen::Affine3d T_world_base = m_robot->get_T_world_frame(m_base_frame);
  const Eigen::Matrix3d R_world_base = T_world_base.linear();

  // Where we want the end effector to be after dt: integrate the commanded twist
  const Eigen::Affine3d T_current = m_robot->get_T_world_frame(m_ee_frame);
  Eigen::Affine3d T_target = T_current;
  T_target.translation() += R_world_base * net_force.head<3>() * dt;
  const Eigen::Vector3d rot_vec = R_world_base * net_force.tail<3>() * dt;
  if (rot_vec.norm() > 1e-9)
  {
    T_target.linear() = Eigen::AngleAxisd(rot_vec.norm(), rot_vec.normalized()) * T_current.linear();
  }
  m_ee_task.set_T_world_frame(T_target);

  if (m_obstacle_constraint)
  {
    const Obstacles & obstacles = *m_obstacles_buffer.readFromRT();
    m_obstacle_constraint->obstacles.resize(obstacles.size());
    for (size_t i = 0; i < obstacles.size(); ++i)
    {
      m_obstacle_constraint->obstacles[i] = obstacles[i];
      m_obstacle_constraint->obstacles[i].T_world_obstacle = T_world_base * obstacles[i].T_world_obstacle;
    }
  }
  if (m_self_constraint)
  {
    for (size_t i = 0; i < m_body_boxes.size(); ++i)
    {
      m_self_constraint->obstacles[i].T_world_obstacle =
        m_robot->get_T_world_frame(m_body_boxes[i].frame_index) * m_body_boxes[i].box.T_world_obstacle;
    }
  }

  try
  {
    // true makes the solver apply the step to the robot's state
    m_solver->solve(true);
  }
  catch (std::runtime_error & e)
  {
    // E.g. an infeasible QP: don't move this step.
    RCLCPP_ERROR_THROTTLE(m_handle->get_logger(), *m_handle->get_clock(), 1000,
                          "PlaCo failed to find a valid solution: %s", e.what());
  }

  for (int i = 0; i < m_number_joints; ++i)
  {
    m_current_positions(i) = m_robot->get_joint(m_joint_names[i]);
    m_current_velocities(i) = dt > 0 ? (m_current_positions(i) - m_last_positions(i)) / dt : 0.0;
  }

  trajectory_msgs::msg::JointTrajectoryPoint control_cmd;
  for (int i = 0; i < m_number_joints; ++i)
  {
    control_cmd.positions.push_back(m_current_positions(i));
    control_cmd.velocities.push_back(m_current_velocities(i));
  }
  control_cmd.time_from_start = period;

  m_last_positions = m_current_positions;
  m_last_velocities = m_current_velocities;

  publishCollisionSpheres();

  return control_cmd;
}

bool PlaCoSolver::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh,
                       const KDL::Chain & chain, const KDL::JntArray & upper_pos_limits,
                       const KDL::JntArray & lower_pos_limits)
{
  IKSolver::init(nh, chain, upper_pos_limits, lower_pos_limits);

  std::string robot_description = nh->get_parameter("robot_description").as_string();
  m_base_frame = nh->get_parameter("robot_base_link").as_string();
  m_ee_frame = chain.segments.back().getName();
  m_joint_names.clear();
  for (const auto & segment : chain.segments)
  {
    if (segment.getJoint().getType() != KDL::Joint::None)
    {
      m_joint_names.push_back(segment.getJoint().getName());
    }
  }

  try
  {
    // Collision meshes aren't used (obstacles use collision spheres), so don't load them.
    m_robot = std::make_unique<placo::model::RobotWrapper>(
      "", placo::model::RobotWrapper::IGNORE_COLLISIONS, robot_description);
    m_solver = std::make_unique<placo::kinematics::KinematicsSolver>(*m_robot);

    // Only the chain's joints move: hold the floating base and every other joint, at zero
    // (clamped to its limits, so the joint limit constraints stay feasible).
    m_solver->mask_fbase(true);
    for (const auto & name : m_robot->joint_names())
    {
      if (std::find(m_joint_names.begin(), m_joint_names.end(), name) != m_joint_names.end())
      {
        continue;
      }
      const int q = m_robot->get_joint_offset(name);
      m_robot->set_joint(name, std::clamp(0.0, m_robot->model.lowerPositionLimit[q],
                                          m_robot->model.upperPositionLimit[q]));
      m_solver->mask_dof(name);
    }
    m_solver->enable_joint_limits(true);
    m_robot->update_kinematics();

    m_ee_task = m_solver->add_frame_task(m_ee_frame, m_robot->get_T_world_frame(m_ee_frame));
    m_ee_task.configure("ee_task", "soft", 1.0, 1.0);

    const auto posture =
      auto_declare<std::vector<double>>("placo.posture.positions", std::vector<double>());
    const double posture_weight = auto_declare<double>("placo.posture.weight", 0.00001);
    if (!posture.empty() && static_cast<int>(posture.size()) != m_number_joints)
    {
      RCLCPP_ERROR(nh->get_logger(), "PlaCo: placo.posture.positions needs %d values, got %zu",
                   m_number_joints, posture.size());
      return false;
    }
    m_posture_task = &m_solver->add_joints_task();
    m_posture_task->configure("posture", "soft", posture_weight);
    m_posture_from_start = posture.empty();
    for (int i = 0; i < m_number_joints; ++i)
    {
      m_posture_task->set_joint(m_joint_names[i], posture.empty() ? 0.0 : posture[i]);
    }
  }
  catch (std::exception & e)
  {
    RCLCPP_ERROR(nh->get_logger(), "PlaCo: failed to set up the robot model: %s", e.what());
    return false;
  }

  return initObstacleAvoidance() && initSelfCollision();
}

bool PlaCoSolver::initObstacleAvoidance()
{
  const auto frames =
    auto_declare<std::vector<std::string>>("placo.collision_spheres.frames", std::vector<std::string>());
  const auto offsets =
    auto_declare<std::vector<double>>("placo.collision_spheres.offsets", std::vector<double>());
  const auto radii = auto_declare<std::vector<double>>("placo.collision_spheres.radii", std::vector<double>());
  const double margin = auto_declare<double>("placo.obstacles.margin", 0.01);
  const double trigger = auto_declare<double>("placo.obstacles.trigger", 0.05);
  const double gain = auto_declare<double>("placo.obstacles.gain", 0.1);
  const bool hard = auto_declare<bool>("placo.obstacles.hard", true);
  const double weight = auto_declare<double>("placo.obstacles.weight", 1000.0);

  if (frames.empty())
  {
    RCLCPP_INFO(m_handle->get_logger(), "PlaCo: no collision spheres, obstacle avoidance is off");
    return true;
  }
  if (offsets.size() != 3 * frames.size() || radii.size() != frames.size())
  {
    RCLCPP_ERROR(m_handle->get_logger(),
                 "PlaCo: placo.collision_spheres needs one radius and three offsets per frame "
                 "(%zu frames, %zu offsets, %zu radii)",
                 frames.size(), offsets.size(), radii.size());
    return false;
  }
  if (gain <= 0.0 || gain > 1.0)
  {
    RCLCPP_ERROR(m_handle->get_logger(), "PlaCo: placo.obstacles.gain must be in (0, 1]");
    return false;
  }

  m_obstacle_constraint = &m_solver->add_constraint(new AvoidObstaclesConstraint());
  m_obstacle_constraint->configure("obstacles", hard ? "hard" : "soft", weight);
  m_obstacle_constraint->margin = margin;
  m_obstacle_constraint->trigger = trigger;
  m_obstacle_constraint->gain = gain;
  for (size_t i = 0; i < frames.size(); ++i)
  {
    CollisionSphere sphere;
    sphere.frame = frames[i];
    try
    {
      sphere.frame_index = m_robot->get_frame_index(frames[i]);
    }
    catch (std::exception & e)
    {
      RCLCPP_ERROR(m_handle->get_logger(), "PlaCo: collision sphere frame '%s' isn't in the URDF",
                   frames[i].c_str());
      return false;
    }
    sphere.offset = Eigen::Vector3d(offsets[3 * i], offsets[3 * i + 1], offsets[3 * i + 2]);
    sphere.radius = radii[i];
    m_obstacle_constraint->spheres.push_back(sphere);
  }

  m_obstacles_buffer.writeFromNonRT(Obstacles());
  m_obstacles_sub = m_handle->create_subscription<visualization_msgs::msg::MarkerArray>(
    m_handle->get_name() + std::string("/obstacles"), 10,
    std::bind(&PlaCoSolver::obstaclesCallback, this, std::placeholders::_1));
  m_spheres_pub = m_handle->create_publisher<visualization_msgs::msg::MarkerArray>(
    m_handle->get_name() + std::string("/collision_spheres"), 1);
  m_spheres_rt_pub =
    std::make_unique<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>>(
      m_spheres_pub);

  RCLCPP_INFO(m_handle->get_logger(),
              "PlaCo: obstacle avoidance with %zu collision spheres, obstacles on %s/obstacles",
              frames.size(), m_handle->get_name());
  return true;
}

bool PlaCoSolver::initSelfCollision()
{
  const auto frames =
    auto_declare<std::vector<std::string>>("placo.self_collision.frames", std::vector<std::string>());
  const auto centers = auto_declare<std::vector<double>>("placo.self_collision.centers", std::vector<double>());
  const auto sizes = auto_declare<std::vector<double>>("placo.self_collision.sizes", std::vector<double>());
  const double margin = auto_declare<double>("placo.self_collision.margin", 0.005);

  if (frames.empty())
  {
    return true;
  }
  if (!m_obstacle_constraint)
  {
    RCLCPP_ERROR(m_handle->get_logger(),
                 "PlaCo: placo.self_collision needs collision spheres (placo.collision_spheres)");
    return false;
  }
  if (centers.size() != 3 * frames.size() || sizes.size() != 3 * frames.size())
  {
    RCLCPP_ERROR(m_handle->get_logger(),
                 "PlaCo: placo.self_collision needs three centers and three sizes per frame "
                 "(%zu frames, %zu centers, %zu sizes)",
                 frames.size(), centers.size(), sizes.size());
    return false;
  }

  for (size_t i = 0; i < frames.size(); ++i)
  {
    BodyBox body_box;
    body_box.frame = frames[i];
    try
    {
      body_box.frame_index = m_robot->get_frame_index(frames[i]);
    }
    catch (std::exception & e)
    {
      RCLCPP_ERROR(m_handle->get_logger(), "PlaCo: self collision frame '%s' isn't in the URDF",
                   frames[i].c_str());
      return false;
    }
    body_box.box.type = Obstacle::Type::Box;
    body_box.box.T_world_obstacle =
      Eigen::Translation3d(centers[3 * i], centers[3 * i + 1], centers[3 * i + 2]) *
      Eigen::Quaterniond::Identity();
    body_box.box.half_extents = 0.5 * Eigen::Vector3d(sizes[3 * i], sizes[3 * i + 1], sizes[3 * i + 2]);
    m_body_boxes.push_back(body_box);
  }

  // Its own constraint, for its own margin: the arm moves close to the body (e.g. the elbow
  // along the torso), closer than it should come to obstacles.
  m_self_constraint = &m_solver->add_constraint(new AvoidObstaclesConstraint());
  m_self_constraint->configure("self_collision", m_obstacle_constraint->priority,
                               m_obstacle_constraint->weight);
  m_self_constraint->margin = margin;
  m_self_constraint->trigger = m_obstacle_constraint->trigger;
  m_self_constraint->gain = m_obstacle_constraint->gain;
  m_self_constraint->spheres = m_obstacle_constraint->spheres;
  for (const auto & body_box : m_body_boxes)
  {
    m_self_constraint->obstacles.push_back(body_box.box);  // Placed in each step
  }

  RCLCPP_INFO(m_handle->get_logger(), "PlaCo: self collision avoidance with %zu boxes",
              m_body_boxes.size());
  return true;
}

void PlaCoSolver::obstaclesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  using visualization_msgs::msg::Marker;
  for (const auto & marker : msg->markers)
  {
    const auto key = std::make_pair(marker.ns, marker.id);
    if (marker.action == Marker::DELETEALL)
    {
      m_obstacle_markers.clear();
      continue;
    }
    if (marker.action == Marker::DELETE)
    {
      m_obstacle_markers.erase(key);
      continue;
    }
    if (!marker.header.frame_id.empty() && marker.header.frame_id != m_base_frame)
    {
      RCLCPP_WARN_THROTTLE(m_handle->get_logger(), *m_handle->get_clock(), 2000,
                           "PlaCo: ignoring obstacle in frame '%s', expected '%s'",
                           marker.header.frame_id.c_str(), m_base_frame.c_str());
      continue;
    }

    Obstacle obstacle;
    if (marker.type == Marker::CUBE)
    {
      obstacle.type = Obstacle::Type::Box;
      obstacle.half_extents = 0.5 * Eigen::Vector3d(marker.scale.x, marker.scale.y, marker.scale.z);
    }
    else if (marker.type == Marker::SPHERE)
    {
      obstacle.type = Obstacle::Type::Sphere;
      obstacle.half_extents = Eigen::Vector3d::Constant(0.5 * marker.scale.x);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(m_handle->get_logger(), *m_handle->get_clock(), 2000,
                           "PlaCo: ignoring obstacle marker of type %d (only CUBE and SPHERE)",
                           marker.type);
      continue;
    }
    const auto & p = marker.pose.position;
    const auto & o = marker.pose.orientation;
    Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    q = q.norm() > 1e-9 ? q.normalized() : Eigen::Quaterniond::Identity();
    obstacle.T_world_obstacle = Eigen::Translation3d(p.x, p.y, p.z) * q;  // In the base frame, for now
    m_obstacle_markers[key] = obstacle;
  }

  Obstacles obstacles;
  for (const auto & entry : m_obstacle_markers)
  {
    obstacles.push_back(entry.second);
  }
  m_obstacles_buffer.writeFromNonRT(obstacles);
}

void PlaCoSolver::publishCollisionSpheres()
{
  if (!m_spheres_rt_pub)
  {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (now - m_last_spheres_publish < std::chrono::milliseconds(100))
  {
    return;
  }
  m_last_spheres_publish = now;

  const Eigen::Affine3d T_base_world = m_robot->get_T_world_frame(m_base_frame).inverse();
  const auto centers = m_obstacle_constraint->sphereCenters();
  visualization_msgs::msg::MarkerArray msg;
  for (size_t i = 0; i < centers.size(); ++i)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = m_base_frame;
    marker.header.stamp = m_handle->now();
    marker.ns = "collision_spheres";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    const Eigen::Vector3d c = T_base_world * centers[i];
    marker.pose.position.x = c.x();
    marker.pose.position.y = c.y();
    marker.pose.position.z = c.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * m_obstacle_constraint->spheres[i].radius;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.a = 0.5;
    msg.markers.push_back(marker);
  }
  for (size_t i = 0; i < m_body_boxes.size(); ++i)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = m_base_frame;
    marker.header.stamp = m_handle->now();
    marker.ns = "self_collision";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    const Eigen::Affine3d T = T_base_world * m_robot->get_T_world_frame(m_body_boxes[i].frame_index) *
                              m_body_boxes[i].box.T_world_obstacle;
    const Eigen::Quaterniond q(T.linear());
    marker.pose.position.x = T.translation().x();
    marker.pose.position.y = T.translation().y();
    marker.pose.position.z = T.translation().z();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 2.0 * m_body_boxes[i].box.half_extents.x();
    marker.scale.y = 2.0 * m_body_boxes[i].box.half_extents.y();
    marker.scale.z = 2.0 * m_body_boxes[i].box.half_extents.z();
    marker.color.b = 1.0;
    marker.color.g = 0.5;
    marker.color.a = 0.3;
    msg.markers.push_back(marker);
  }
  m_spheres_rt_pub->try_publish(msg);
}
}  // namespace cartesian_controller_base

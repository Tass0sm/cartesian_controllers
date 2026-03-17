////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_compliance_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_compliance_controller/cartesian_compliance_controller.h>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mutex>

namespace cartesian_compliance_controller
{
CartesianComplianceController::CartesianComplianceController()
// Base constructor won't be called in diamond inheritance, so call that
// explicitly
: Base::CartesianControllerBase(),
  MotionBase::CartesianMotionController(),
  ForceBase::CartesianForceController()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianComplianceController::on_init()
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_init() != TYPE::SUCCESS || ForceBase::on_init() != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  auto node = get_node();

  node->declare_parameter<std::string>("compliance_ref_link", "");

  constexpr double default_lin_stiff = 500.0;
  constexpr double default_rot_stiff = 50.0;
  node->declare_parameter<double>("stiffness.trans_x", default_lin_stiff);
  node->declare_parameter<double>("stiffness.trans_y", default_lin_stiff);
  node->declare_parameter<double>("stiffness.trans_z", default_lin_stiff);
  node->declare_parameter<double>("stiffness.rot_x", default_rot_stiff);
  node->declare_parameter<double>("stiffness.rot_y", default_rot_stiff);
  node->declare_parameter<double>("stiffness.rot_z", default_rot_stiff);

  // Damping parameters for velocity feedforward
  // net_force = K_stiff * (x_target - x_current) + K_damp * (v_target - v_current)
  constexpr double default_lin_damp = 10.0;
  constexpr double default_rot_damp = 1.0;
  node->declare_parameter<double>("damping.trans_x", default_lin_damp);
  node->declare_parameter<double>("damping.trans_y", default_lin_damp);
  node->declare_parameter<double>("damping.trans_z", default_lin_damp);
  node->declare_parameter<double>("damping.rot_x", default_rot_damp);
  node->declare_parameter<double>("damping.rot_y", default_rot_damp);
  node->declare_parameter<double>("damping.rot_z", default_rot_damp);

  // Subscribe to target twist from MPC node
  m_target_twist_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
    get_node()->get_name() + std::string("/target_twist"),
    10,
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(m_twist_mutex);
      m_target_twist[0] = msg->twist.linear.x;
      m_target_twist[1] = msg->twist.linear.y;
      m_target_twist[2] = msg->twist.linear.z;
      m_target_twist[3] = msg->twist.angular.x;
      m_target_twist[4] = msg->twist.angular.y;
      m_target_twist[5] = msg->twist.angular.z;
      m_twist_received = true;
    });

  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianComplianceController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_configure(previous_state) != TYPE::SUCCESS ||
      ForceBase::on_configure(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  // make sure compliance link is part of the robot chain
  m_compliance_ref_link = get_node()->get_parameter("compliance_ref_link").as_string();
  if (!Base::robotChainContains(m_compliance_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_compliance_ref_link
                                                    << " is not part of the kinematic chain from "
                                                    << Base::m_robot_base_link << " to "
                                                    << Base::m_end_effector_link);
    return TYPE::ERROR;
  }

  // make sure sensor wrenches are interpreted correctly
  ForceBase::setFtSensorReferenceFrame(m_compliance_ref_link);

  // cache the stiffness and damping matrices, avoid parameter lookups at 250Hz
  ctrl::Vector6D stiff_tmp;
  stiff_tmp[0] = get_node()->get_parameter("stiffness.trans_x").as_double();
  stiff_tmp[1] = get_node()->get_parameter("stiffness.trans_y").as_double();
  stiff_tmp[2] = get_node()->get_parameter("stiffness.trans_z").as_double();
  stiff_tmp[3] = get_node()->get_parameter("stiffness.rot_x").as_double();
  stiff_tmp[4] = get_node()->get_parameter("stiffness.rot_y").as_double();
  stiff_tmp[5] = get_node()->get_parameter("stiffness.rot_z").as_double();
  m_stiffness = stiff_tmp.asDiagonal();

  ctrl::Vector6D damp_tmp;
  damp_tmp[0] = get_node()->get_parameter("damping.trans_x").as_double();
  damp_tmp[1] = get_node()->get_parameter("damping.trans_y").as_double();
  damp_tmp[2] = get_node()->get_parameter("damping.trans_z").as_double();
  damp_tmp[3] = get_node()->get_parameter("damping.rot_x").as_double();
  damp_tmp[4] = get_node()->get_parameter("damping.rot_y").as_double();
  damp_tmp[5] = get_node()->get_parameter("damping.rot_z").as_double();
  m_damping = damp_tmp.asDiagonal();

  RCLCPP_INFO(get_node()->get_logger(),
    "Compliance controller configured with damping: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
    damp_tmp[0], damp_tmp[1], damp_tmp[2], damp_tmp[3], damp_tmp[4], damp_tmp[5]);

  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianComplianceController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // Base::on_activation(..) will get called twice, but that's fine.
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_activate(previous_state) != TYPE::SUCCESS ||
      ForceBase::on_activate(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  // Reset twist state on activation
  {
    std::lock_guard<std::mutex> lock(m_twist_mutex);
    m_target_twist = ctrl::Vector6D::Zero();
    m_twist_received = false;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Compliance controller activated, waiting for target_twist...");

  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianComplianceController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_deactivate(previous_state) != TYPE::SUCCESS ||
      ForceBase::on_deactivate(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }
  return TYPE::SUCCESS;
}

controller_interface::return_type CartesianComplianceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // Control the robot motion in such a way that the resulting net force
  // vanishes. This internal control needs some simulation time steps.
  for (int i = 0; i < Base::m_iterations; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    auto internal_period = rclcpp::Duration::from_seconds(0.02);

    // Compute the net force
    ctrl::Vector6D error = computeComplianceError();

    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(error, internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianComplianceController::computeComplianceError()
{
  // position/orientation error scaled by stiffness, now transformed to base link
  ctrl::Vector6D net_force =
    Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) * MotionBase::computeMotionError()
    + ForceBase::computeForceError();

  // thread-safe copy of target twist and received flag
  ctrl::Vector6D target_twist_copy;
  bool twist_valid;
  {
    std::lock_guard<std::mutex> lock(m_twist_mutex);
    target_twist_copy = m_target_twist;
    twist_valid = m_twist_received;
  }

  // only apply velocity feedforward after receiving first twist message
  // this prevents braking force from zero-initialized target twist
  if (twist_valid)
  {
    ctrl::Vector6D current_vel = Base::m_ik_solver->getEndEffectorVel();
    ctrl::Vector6D vel_error = target_twist_copy - current_vel;

    // transform damping to base link frame, its same as stiffness
    net_force += Base::displayInBaseLink(m_damping, m_compliance_ref_link) * vel_error;
  }

  return net_force;
}

}  // namespace cartesian_compliance_controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_compliance_controller::CartesianComplianceController,
                       controller_interface::ControllerInterface)
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>
#include <string>
#include <limits>
#include <vector>
#include "nav2_controller/plugins/follow_pose_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

FollowPoseGoalChecker::FollowPoseGoalChecker()
: xy_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  stateful_(true),
  check_xy_(true),
  xy_goal_tolerance_sq_(0.0625)
{
}

void FollowPoseGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  plugin_name_ = plugin_name;

  node_ = parent;
  auto node = node_.lock();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".stateful", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&FollowPoseGoalChecker::dynamicParametersCallback, this, _1));
    
  follow_pose_start_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "follow_path_start_pose", rclcpp::SystemDefaultsQoS(),
    std::bind(&FollowPoseGoalChecker::startPoseCallback, this, _1));

}

void FollowPoseGoalChecker::reset()
{
  check_xy_ = true;
}

bool FollowPoseGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & /* goal_pose */,
  const geometry_msgs::msg::Twist &)
{
  geometry_msgs::msg::PoseStamped start_pose;
  {
    std::lock_guard<std::mutex> lock(start_pose_mutex_);
    start_pose = start_pose_;
  }
  float dist_robot_path = std::hypot(query_pose.position.x - start_pose.pose.position.x,
                query_pose.position.y - start_pose.pose.position.y);
  {
    auto node = node_.lock();
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000,
      "plugin [%s], dist_robot_path: %.2f, dist_tolerance_: %.2f",
      plugin_name_.data(), dist_robot_path, dist_tolerance_);
  }
  if (dist_robot_path >= dist_tolerance_) {
    auto node = node_.lock();
    RCLCPP_INFO(
      node->get_logger(),
      "plugin [%s], Goal reached, dist_robot_path: %.2f, dist_tolerance_: %.2f",
      plugin_name_.data(), dist_robot_path, dist_tolerance_);
    return true;
  }

  return false;
}

bool FollowPoseGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

rcl_interfaces::msg::SetParametersResult
FollowPoseGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".xy_goal_tolerance") {
        xy_goal_tolerance_ = parameter.as_double();
        xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
      } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

void FollowPoseGoalChecker::startPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (msg) {
    std::lock_guard<std::mutex> lock(start_pose_mutex_);
    start_pose_ = *msg;
  }
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::FollowPoseGoalChecker, nav2_core::GoalChecker)

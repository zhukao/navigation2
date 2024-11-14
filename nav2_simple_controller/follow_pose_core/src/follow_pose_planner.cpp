// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "follow_pose_core/follow_pose_planner.hpp"
#include "follow_pose_core/exceptions.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace follow_pose_core
{

// 以c为中心，pt_start逆时针旋转到pt_end的弧度
// 返回值范围 [0, 2.0 * M_PI]
float CalAngelOfTwoVector(float c_x, float c_y,
                        float pt_start_x, float pt_start_y,
                        float pt_end_x, float pt_end_y) {
  float theta =
      atan2(pt_start_x - c_x, pt_start_y - c_y) - atan2(pt_end_x - c_x, pt_end_y - c_y);
  if (theta < 0) theta = theta + 2.0 * M_PI;
  return theta;
}

FollowPosePlanner::FollowPosePlanner()
{
  RCLCPP_INFO(rclcpp::get_logger("FollowPosePlanner"), "Constructing FollowPosePlanner");
}

void FollowPosePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;

  plugin_name_ = name;

  RCLCPP_INFO(logger_, "config plugin: %s", name.data());

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance",
    rclcpp::ParameterValue(0.1));

  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
  RCLCPP_INFO(logger_, "Setting transform_tolerance to %f", transform_tolerance);

  last_compute_cmd_time_ = node->now();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  follow_pose_start_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "follow_path_start_pose", rclcpp::SystemDefaultsQoS());

  dynamic_obstacle_markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    pub_dynamic_obstacle_markers_topic_name_,
    rclcpp::QoS(10));
  InitMarker(marker_);

}

void
FollowPosePlanner::activate()
{
}

void
FollowPosePlanner::deactivate()
{
}

void
FollowPosePlanner::cleanup()
{

}

void
FollowPosePlanner::setPlan(const nav_msgs::msg::Path & path)
{
  current_path_ = path;
  auto path2d = nav_2d_utils::pathToPath2D(path);
  global_plan_ = path2d;

  RCLCPP_INFO(logger_, "plugin [%s] recved global plan frame id: %s",
    plugin_name_.data(), global_plan_.header.frame_id.c_str());

  if (!sp_last_path_end_pose_) {
    sp_last_path_end_pose_ =
      std::make_shared<geometry_msgs::msg::PoseStamped>(path.poses.back());
    path_end_pose_updated_ = true;
  } else {
    float dist = std::hypot(path.poses.back().pose.position.x - sp_last_path_end_pose_->pose.position.x,
      path.poses.back().pose.position.y - sp_last_path_end_pose_->pose.position.y);
    if (dist > 0.05) {
      path_end_pose_updated_ = true;
      *sp_last_path_end_pose_ = path.poses.back();
    } else {
      path_end_pose_updated_ = false;
    }
  }
}

geometry_msgs::msg::TwistStamped
FollowPosePlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /* goal_checker */)
{
  RCLCPP_DEBUG(logger_, "plugin: %s doing computeVelocityCommands", plugin_name_.data());

  auto node = node_.lock();
  if ((node->now() - last_compute_cmd_time_).seconds() > 1.0 || path_end_pose_updated_) {
    RCLCPP_INFO(logger_, "Update robot start pose: %.2f, %.2f", pose.pose.position.x, pose.pose.position.y);
    start_robot_pose_ = pose;
    path_end_pose_updated_ = false;
  }
  last_compute_cmd_time_ = node->now();

  if (follow_pose_start_pose_pub_) {
    follow_pose_start_pose_pub_->publish(start_robot_pose_);
  }

  nav_2d_msgs::msg::Twist2DStamped cmd_vel2d = computeVelocityCommands(
    nav_2d_utils::poseStampedToPose2D(pose),
    nav_2d_utils::twist3Dto2D(velocity));
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.twist = nav_2d_utils::twist2Dto3D(cmd_vel2d.velocity);
  return cmd_vel;
}

void
FollowPosePlanner::prepareGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose, nav_2d_msgs::msg::Path2D & transformed_plan,
  nav_2d_msgs::msg::Pose2DStamped & goal_pose, bool publish_plan)
{
  transformed_plan = transformGlobalPlan(pose);
  if (publish_plan) {
  }

  goal_pose.header.frame_id = global_plan_.header.frame_id;
  goal_pose.pose = global_plan_.poses.back();
  nav_2d_utils::transformPose(
    tf_, costmap_ros_->getGlobalFrameID(), goal_pose,
    goal_pose, transform_tolerance_);
}

nav_2d_msgs::msg::Twist2DStamped
FollowPosePlanner::computeVelocityCommands(
  const nav_2d_msgs::msg::Pose2DStamped & pose,
  const nav_2d_msgs::msg::Twist2D & velocity)
{
  RCLCPP_DEBUG(logger_, "plugin [%s] robot pose frame_id: %s, velocity: %.2f, %.2f",
    plugin_name_.data(), pose.header.frame_id.data(), velocity.x, velocity.y);

  geometry_msgs::msg::PoseStamped robot_pose = nav_2d_utils::pose2DToPoseStamped(pose);
      
  // nav_2d_msgs::msg::Path2D transformed_plan;
  // nav_2d_msgs::msg::Pose2DStamped goal_pose;

  // prepareGlobalPlan(pose, transformed_plan, goal_pose);

  // nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  // std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  
  nav_2d_msgs::msg::Twist2DStamped cmd_vel;
  bool take_over_continue = RotateAndMoveOnce(3.1415 / 180.0 * 10.0, 0.3, start_robot_pose_, cmd_vel, robot_pose);
  // 返回true表示需要继续接管
  if (take_over_continue) {
  } else {
    // 停止接管
    RCLCPP_WARN(logger_, "take over completed");
    ClearMarker("TakeOver");
    std::unique_lock<std::mutex> lock(mtx_nav_state_);
    // nav_state_ = NavState::TAKE_OVER_SUCCED;

    throw nav2_core::PlannerException("Failed to follow pose");
  }

  return cmd_vel;
}

void FollowPosePlanner::setSpeedLimit(const double & speed_limit, const bool & percentage) {
  RCLCPP_DEBUG(
    logger_, "Setting speed limit to %f %s", speed_limit,
    percentage ? "as a percentage of maximum" : "as absolute speed");
}

nav_2d_msgs::msg::Path2D
FollowPosePlanner::transformGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  nav_2d_msgs::msg::Pose2DStamped robot_pose;
  if (!nav_2d_utils::transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // we'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  // If prune_plan is enabled (it is by default) then we want to restrict the
  // plan to distances within that range as well.
  double prune_dist = prune_distance_;

  // Set the maximum distance we'll include points before getting to the part
  // of the path where the robot is located (the start of the plan). Basically,
  // these are the points the robot has already passed.
  double transform_start_threshold;
  if (prune_plan_) {
    transform_start_threshold = std::min(dist_threshold, prune_dist);
  } else {
    transform_start_threshold = dist_threshold;
  }

  // Set the maximum distance we'll include points after the part of the plan
  // near the robot (the end of the plan). This determines the amount of the
  // plan passed on to the critics
  double transform_end_threshold;
  double forward_prune_dist = forward_prune_distance_;
  if (shorten_transformed_plan_) {
    transform_end_threshold = std::min(dist_threshold, forward_prune_dist);
  } else {
    transform_end_threshold = dist_threshold;
  }

  // Find the first pose in the global plan that's further than prune distance
  // from the robot using integrated distance
  auto prune_point = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), prune_dist);

  // Find the first pose in the plan (upto prune_point) that's less than transform_start_threshold
  // from the robot.
  auto transformation_begin = std::find_if(
    begin(global_plan_.poses), prune_point,
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose.pose, global_plan_pose) < transform_start_threshold;
    });

  // Find the first pose in the end of the plan that's further than transform_end_threshold
  // from the robot using integrated distance
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose.pose) > transform_end_threshold;
    });

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_2d_msgs::msg::Path2D transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Helper function for the transform below. Converts a pose2D from global
  // frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      nav_2d_msgs::msg::Pose2DStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.pose = global_plan_pose;
      nav_2d_utils::transformPose(
        tf_, transformed_plan.header.frame_id,
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose.pose;
    };

  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration.
  if (prune_plan_) {
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  }

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }
  return transformed_plan;
}


// 返回true表示需要继续接管
bool FollowPosePlanner::RotateAndMoveOnce(float yaw_goal_tolerance, float stop_dist_thr,
  geometry_msgs::msg::PoseStamped start_robot_pose, nav_2d_msgs::msg::Twist2DStamped& cmd_vel,
  const geometry_msgs::msg::PoseStamped robot_current_pose) {
    
  // init with zero
  geometry_msgs::msg::Twist twist_3d;
  twist_3d.angular.x = 0;
  twist_3d.angular.y = 0;
  twist_3d.angular.z = 0;
  twist_3d.linear.x = 0;
  twist_3d.linear.y = 0;
  twist_3d.linear.z = 0;
  cmd_vel.velocity = nav_2d_utils::twist3Dto2D(twist_3d);

  nav_msgs::msg::Path current_path = current_path_;

  // 寻找和robot距离为 dist_robot_path_thr 的最近路径规划点
  float dist_robot_path_thr = stop_dist_thr;
  int path_index = FindPathIndex(current_path, dist_robot_path_thr, robot_current_pose);
  if (path_index < 0) {
    RCLCPP_ERROR(logger_, "find path index fail");
    return false;
  }

  auto get_distance = [this, robot_current_pose](geometry_msgs::msg::PoseStamped dest_pose)->float{
    // 计算robot和path之间的距离
    float dist = std::hypot(robot_current_pose.pose.position.x - dest_pose.pose.position.x,
                  robot_current_pose.pose.position.y - dest_pose.pose.position.y);
    RCLCPP_DEBUG(logger_, "robot pose frame_id: %s, (%.2f, %.2f), ts: %d.%d, dest (%.2f, %.2f), dist: %.2f",
      robot_current_pose.header.frame_id.data(),
      robot_current_pose.pose.position.x, robot_current_pose.pose.position.y,
      robot_current_pose.header.stamp.sec, robot_current_pose.header.stamp.nanosec,
      dest_pose.pose.position.x, dest_pose.pose.position.y,
      dist
      );
      
    return dist;
  };
    
  {
    std::unique_lock<std::mutex> lock(mtx_nav_state_);
    // if (nav_state_ != NavState::TAKE_OVER_GOING) {
    //   RCLCPP_WARN(logger_, "nav state changed to %d, stop take-over task", static_cast<int>(nav_state_));
    //   // 状态已经发生变化，停止接管
    //   // publishZeroVelocity();
    //   return false;
    // }
  }

  // 计算robot当前位置和移动前位置之间的距离
  float dist_robot_to_start = get_distance(start_robot_pose);
  // if (dist_robot_to_start < 0) {
  //   // 计算失败
  //   return false;
  // }
  // RCLCPP_DEBUG(logger_,
  //   "dist_robot_to_start: %.2f, stop_dist_thr: %.2f",
  //   dist_robot_to_start, stop_dist_thr);
  // if (dist_robot_to_start >= stop_dist_thr) {
  //   // 移动了足够远的距离，停止
  //   RCLCPP_INFO(logger_,
  //     "dist_robot_to_start: %.2f, stop_dist_thr: %.2f",
  //     dist_robot_to_start, stop_dist_thr);
  //   RCLCPP_INFO(logger_, "no need to move");
  //   // 不需要旋转和平移，继续跑轨迹规划
  //   // publishZeroVelocity();
  //   return false;
  // }

  float dyaw;
  if (!GetYawDiff(current_path.poses.at(path_index), current_path.header.frame_id, dyaw, robot_current_pose)) {
    return false;
  }
  // 将逆时针弧度转成最小弧度夹角
  float yaw_absolute = dyaw;
  if (dyaw > 3.14159) {
    yaw_absolute = 2 * 3.14159 - dyaw;
  }
  // 如果robot和目标位置距离很近，不需要再rotate
  float dist_robot_to_path = get_distance(current_path.poses.at(path_index));
  RCLCPP_DEBUG(logger_, "yaw_absolute: %.2f, %d, yaw_goal_tolerance: %.2f, %d, dist_robot_to_path: %.2f",
    yaw_absolute, static_cast<int>(yaw_absolute * 180.0 / 3.14159),
    yaw_goal_tolerance, static_cast<int>(yaw_goal_tolerance * 180.0 / 3.14159),
    dist_robot_to_path);
  if (yaw_absolute < yaw_goal_tolerance || dist_robot_to_path < 0.1) {
    RCLCPP_DEBUG(logger_, "do not need to rotate");
  } else {
    twist_3d.linear.x = 0;
    twist_3d.linear.y = 0;
    twist_3d.linear.z = 0;
    twist_3d.angular.x = 0.0;
    twist_3d.angular.y = 0;
    if (dyaw < 3.14159) {
      // 逆时针旋转
      twist_3d.angular.z = (1.0) * rotate_z;
      RCLCPP_DEBUG(logger_, "rotate anti-clockwise");
    } else {
      // 顺时针旋转
      twist_3d.angular.z = (-1.0) * rotate_z;
      RCLCPP_DEBUG(logger_, "rotate clockwise");
    }

    // 如果角度很小，减小旋转速度
    if (yaw_absolute < yaw_goal_tolerance * 2.0) {
      // 转动了超过一半
      twist_3d.angular.z = twist_3d.angular.z * 0.5;
    }

    cmd_vel.velocity = nav_2d_utils::twist3Dto2D(twist_3d);
    // publishVelocity(cmd_vel_2d);
    return true;
  }

  // 运行到这里说明角度小于阈值

  // 移动
  // 计算每个规划点对应cell的cost，判断是否为有效路径
  if (!CheckPathValid(current_path, path_index * 2)) {
    RCLCPP_ERROR(logger_, "path is not valid, which is in obstacle");
    return false;
  }

  // 计算robot当前位置和移动前位置之间的距离
  dist_robot_to_start = get_distance(start_robot_pose);
  if (dist_robot_to_start < 0) {
    // 计算失败
    return false;
  }
  RCLCPP_DEBUG(logger_,
    "dist_robot_to_start: %.2f, stop_dist_thr: %.2f",
    dist_robot_to_start, stop_dist_thr);

  // if (dist_robot_to_start >= stop_dist_thr) {
  //   // 移动了足够远的距离，停止
  //   RCLCPP_INFO(logger_,
  //     "yaw_absolute: %.2f, %d, yaw_goal_tolerance: %.2f, %d, dist_robot_to_start: %.2f, stop_dist_thr: %.2f",
  //     yaw_absolute, static_cast<int>(yaw_absolute * 180.0 / 3.14159),
  //     yaw_goal_tolerance, static_cast<int>(yaw_goal_tolerance * 180.0 / 3.14159),
  //     dist_robot_to_start, stop_dist_thr);

  //   RCLCPP_INFO(logger_, "no need to move");
  //   // 不需要旋转和平移，继续跑轨迹规划
  //   // publishZeroVelocity();
  //   return false;
  // } else 
  {
    RCLCPP_DEBUG(logger_, "do move");
    twist_3d.linear.x = 0.2;
    twist_3d.linear.y = 0;
    twist_3d.linear.z = 0;
    twist_3d.angular.x = 0.0;
    twist_3d.angular.y = 0;
    twist_3d.angular.z = 0;
    if (dist_robot_to_start >= stop_dist_thr * 0.5) {
      // 移动了超过一半
      twist_3d.linear.x = twist_3d.linear.x * 0.5;
    }

    // publishVelocity(cmd_vel_2d);
    cmd_vel.velocity = nav_2d_utils::twist3Dto2D(twist_3d);
    return true;
  }

  return false;
}

void FollowPosePlanner::InitMarker(visualization_msgs::msg::Marker& marker) {
  // https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html
  marker.ns = "CUBE";
  marker.id = 0;
  // Set the marker type.
  marker.type = visualization_msgs::msg::Marker::CUBE;
  // Set the marker action.  Options are ADD DELETE DELETEALL
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Marker group position and orientation
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.01;

  marker.lifetime = builtin_interfaces::msg::Duration();  // 0 - unlimited
  // marker.lifetime.sec = 5.0;
  // marker.lifetime.nanosec = 0.0;
  
}

void FollowPosePlanner::ClearMarker(std::string ns) {
  if (dynamic_obstacle_markers_pub_) {
    visualization_msgs::msg::MarkerArray marker_array_clear;
    auto marker = marker_;
    marker.id = 0;
    marker.ns = ns;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array_clear.markers.push_back(marker);
    dynamic_obstacle_markers_pub_->publish(std::move(marker_array_clear));
  }
}


// 根据规划的路径current_path，寻找第一个和robot距离超过dist_robot_path_thr的规划点
// 返回对应的规划点的索引idx
int FollowPosePlanner::FindPathIndex(const nav_msgs::msg::Path& current_path,
  float dist_robot_path_thr, const geometry_msgs::msg::PoseStamped robot_current_pose) {
  if (current_path.poses.empty()) {
    RCLCPP_WARN(logger_, "Path is empty");
    return -1;
  }

  last_recved_global_path_sec_ = current_path.header.stamp.sec;

  RCLCPP_DEBUG(logger_, "robot pose ts: %d.%d, current_path ts: %d.%d",
    robot_current_pose.header.stamp.sec, robot_current_pose.header.stamp.nanosec,
    current_path.header.stamp.sec, current_path.header.stamp.nanosec
    );

  std::string path_frame = current_path.header.frame_id;
  std::string robot_frame = robot_current_pose.header.frame_id;
  double robot_x = robot_current_pose.pose.position.x;
  double robot_y = robot_current_pose.pose.position.y;
  if (path_frame != robot_frame) {
    try {
      geometry_msgs::msg::Transform transform_robot2path =
        tf_buffer_->lookupTransform(path_frame, robot_frame,
        robot_current_pose.header.stamp, tf2::durationFromSec(0.1)).transform;
      tf2::Transform tf2_transform_robot2path;
      tf2::fromMsg(transform_robot2path, tf2_transform_robot2path);
      
      tf2::Vector3 p(robot_current_pose.pose.position.x, robot_current_pose.pose.position.y, 0);
      p = tf2_transform_robot2path * p;
      robot_x = p.x();
      robot_y = p.y();
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_,
        "Could not transform %s to %s: %s",
        robot_frame.c_str(), path_frame.c_str(), ex.what());
      return -1;
    }
  }

  // 先找到和robot距离最近的点，作为起点
  // 避免因为robot移动了一段距离后，而plan未更新，导致robot返回起点
  int dist_min_path_index = 0;
  float dist_min = std::numeric_limits<float>::max();
  for (size_t idx = 0; idx < current_path.poses.size(); idx++) {
    const auto& path_pose = current_path.poses.at(idx);
    // 计算robot和path之间的距离
    float dist_robot_path = std::hypot(robot_x - path_pose.pose.position.x,
                  robot_y - path_pose.pose.position.y);
    if (dist_robot_path < dist_min) {
      dist_min = dist_robot_path;
      dist_min_path_index = idx;
    }
  }

  // TODO
  // 对 current_path 的时间戳和 dist_min 进行校验，如果大于阈值，说明是过期的路径规划

  // 找到第一个和robot之间距离超过阈值的规划点
  size_t path_index = dist_min_path_index;
  // robot和规划点之间的距离
  float dist_robot_path;
  for (size_t idx = dist_min_path_index; idx < current_path.poses.size(); idx++) {
    const auto& path_pose = current_path.poses.at(idx);
    // 计算robot和path之间的距离
    dist_robot_path = std::hypot(robot_x - path_pose.pose.position.x,
                  robot_y - path_pose.pose.position.y);
    if (dist_robot_path > dist_robot_path_thr) {
      path_index = idx;
      break;
    }
  }
  
  if (dist_robot_path <= dist_robot_path_thr) {
    RCLCPP_ERROR(logger_, "Find the path point fail, dist_robot_path: %.2f, dist_robot_path_thr: %.2f",
      dist_robot_path, dist_robot_path_thr);
    return -1;
  }

  RCLCPP_DEBUG(logger_, "Find the path point success, robot frame: %s, (%.2f, %.2f), dist_min_path_index: %d, dist_min: %.2f, path frame: %s, (%.2f, %.2f), index: %d, dist_robot_path: %.2f, dist_robot_path_thr: %.2f",
    robot_current_pose.header.frame_id.c_str(),
    robot_current_pose.pose.position.x, robot_current_pose.pose.position.y,
    dist_min_path_index, dist_min,
    current_path.header.frame_id.c_str(),
    current_path.poses.at(path_index).pose.position.x, current_path.poses.at(path_index).pose.position.y,
    static_cast<int>(path_index), dist_robot_path,
    dist_robot_path_thr
    );
  
  // {
  //   // 发布找到的规划点
  //   visualization_msgs::msg::MarkerArray marker_array;
  //   auto marker = marker_;
  //   int count = 0;
  //   marker.header = current_path.header;
  //   marker.id = count++;
  //   marker.ns = "CUBE";
  //   marker.pose.position.x = current_path.poses.at(path_index).pose.position.x;
  //   marker.pose.position.y = current_path.poses.at(path_index).pose.position.y;
  //   marker.action = visualization_msgs::msg::Marker::ADD;
  //   marker_array.markers.push_back(marker);
  //   if (marker_array.markers.size() > 0) {
  //     dynamic_obstacle_markers_pub_->publish(std::move(marker_array));
  //   }
  // }

  return static_cast<int>(path_index);
}

bool FollowPosePlanner::GetYawDiff(
  const geometry_msgs::msg::PoseStamped& path_pose,
  std::string path_frame, float& dyaw,
  const geometry_msgs::msg::PoseStamped robot_current_pose) {
  std::string robot_frame = robot_current_pose.header.frame_id;
  std::string base_link_frame_id = "base_link";

  tf2::Transform tf2_transform_robot2baselink;
  tf2::Transform tf2_transform_path2baselink;
  try {
    geometry_msgs::msg::Transform transform_robot2baselink =
      tf_buffer_->lookupTransform(base_link_frame_id, robot_frame,
      robot_current_pose.header.stamp, tf2::durationFromSec(0.1)).transform;
    tf2::fromMsg(transform_robot2baselink, tf2_transform_robot2baselink);
    
    geometry_msgs::msg::Transform transform_path2baselink =
      tf_buffer_->lookupTransform(base_link_frame_id, path_frame,
      path_pose.header.stamp, tf2::durationFromSec(0.1)).transform;
    tf2::fromMsg(transform_path2baselink, tf2_transform_path2baselink);
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(logger_, "Can't transform %s or %s to %s: %s",
      robot_frame.data(), path_frame.data(), base_link_frame_id.data(),ex.what());
    return false;
  }

  double robot_x;
  double robot_y;
  {
    tf2::Vector3 p(robot_current_pose.pose.position.x, robot_current_pose.pose.position.y, 0);
    p = tf2_transform_robot2baselink * p;
    robot_x = p.x();
    robot_y = p.y();
  }
  
  double path_x = 0;
  double path_y = 0;
  {
    tf2::Vector3 p(path_pose.pose.position.x, path_pose.pose.position.y, 0);
    p = tf2_transform_path2baselink * p;
    path_x = p.x();
    path_y = p.y();
  }

  // 计算robot和path之间的距离
  double step = std::hypot(robot_x - path_x, robot_y - path_y);
  double robot_forward_x = robot_x + step;
  double robot_forward_y = robot_y;

  dyaw = CalAngelOfTwoVector(robot_x, robot_y,
                      robot_forward_x, robot_forward_y,
                      path_x, path_y);

  RCLCPP_DEBUG(logger_, "dyaw: %.2f, %d, robot at frame: %s, (%.2f, %.2f); frame: %s, (%.2f, %.2f), forward (%.2f, %.2f), path (%.2f, %.2f); path at frame: %s, (%.2f, %.2f)",
    dyaw, static_cast<int>(dyaw * 180.0 / 3.14159),
    robot_current_pose.header.frame_id.data(), robot_current_pose.pose.position.x, robot_current_pose.pose.position.y,
    base_link_frame_id.data(),
    robot_x, robot_y, robot_forward_x, robot_forward_y, path_x, path_y,
    path_frame.data(), path_pose.pose.position.x, path_pose.pose.position.y
    );
    
  // 发布用于计算的三个点
  if (dynamic_obstacle_markers_pub_)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int count = 0;
    auto marker = marker_;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "TakeOver";
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.01;
    marker.header = robot_current_pose.header;

    // {
    //   // 原始坐标系下的规划点
    //   marker.header.frame_id = path_frame;
    //   marker.color.r = 1.0;
    //   marker.color.g = 1.0;
    //   marker.color.b = 1.0;
    //   marker.color.a = 1.0;
    //   marker.id = count++;
    //   marker.pose.position.x = path_pose.pose.position.x;
    //   marker.pose.position.y = path_pose.pose.position.y;
    //   marker_array.markers.push_back(marker);
    // }

    marker.header.frame_id = base_link_frame_id;
    {
      // robot
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.id = count++;
      marker.pose.position.x = robot_x;
      marker.pose.position.y = robot_y;
      marker_array.markers.push_back(marker);
    }
    {
      // robot正前方点
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.id = count++;
      marker.pose.position.x = robot_forward_x;
      marker.pose.position.y = robot_forward_y;
      marker_array.markers.push_back(marker);
    }
    {
      // 规划点
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker.id = count++;
      marker.pose.position.x = path_x;
      marker.pose.position.y = path_y;
      marker_array.markers.push_back(marker);
    }
    if (marker_array.markers.size() > 0) {
      dynamic_obstacle_markers_pub_->publish(std::move(marker_array));
    }
  }


  return true;
}

bool FollowPosePlanner::CheckPathValid(const nav_msgs::msg::Path& path, int path_index) {
  nav2_costmap_2d::Costmap2D * costmap_ = costmap_ros_->getCostmap();
  if (!costmap_) return false;
  unsigned int cell_x, cell_y;
  for (size_t idx = 0; idx < path.poses.size(); idx++) {
    const auto& pose = path.poses.at(idx);
    if (path_index > 0 && static_cast<int>(idx) >= path_index) {
      break;
    }

    if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, cell_x, cell_y)) {
      RCLCPP_WARN(logger_, "Gloabl plan at (%.2f, %.2f) is outside of grid.",
        pose.pose.position.x, pose.pose.position.y);
      continue;
    }
    unsigned char cost = costmap_->getCost(cell_x, cell_y);
    // TODO
    // 卡个阈值？
    // 使用 CostmapTopicCollisionChecker 判断碰撞
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE
    //  || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE
     ) {
      RCLCPP_WARN(logger_, "Global plan at (%.2f, %.2f) is in an obstacle, cost %d.",
        pose.pose.position.x, pose.pose.position.y, cost);
      // 发布marker
      if (dynamic_obstacle_markers_pub_) {
        visualization_msgs::msg::MarkerArray marker_array;
        auto marker = marker_;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = "CUBE";
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.01;
        marker.header = path.header;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.id = 0;
        marker.pose.position.x = pose.pose.position.x;
        marker.pose.position.y = pose.pose.position.y;
        marker.lifetime.sec = 3.0;
        marker.lifetime.nanosec = 0.0;
        marker_array.markers.push_back(marker);
        dynamic_obstacle_markers_pub_->publish(std::move(marker_array));
      }
      return false;
    }
  }

  return true;
}


}  // namespace follow_pose_core

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  follow_pose_core::FollowPosePlanner,
  nav2_core::Controller)

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

#ifndef FOLLOW_POSE_CORE__FOLLOW_POSE_PLANNER_HPP_
#define FOLLOW_POSE_CORE__FOLLOW_POSE_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
// Rviz
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// Messages
#include <std_msgs/msg/color_rgba.hpp>

namespace follow_pose_core
{

/**
 * @class FollowPosePlanner
 * @brief Plugin-based flexible controller
 */
class FollowPosePlanner : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor that brings up pluginlib loaders
   */
  FollowPosePlanner();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  virtual ~FollowPosePlanner() {}

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief nav2_core computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * This is mostly a wrapper for the protected computeVelocityCommands
   * function which has additional debugging info.
   *
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return The best command for the robot to drive
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param results   Output param, if not NULL, will be filled in with full evaluation results
   * @return          Best command
   */
  virtual nav_2d_msgs::msg::Twist2DStamped computeVelocityCommands(
    const nav_2d_msgs::msg::Pose2DStamped & pose,
    const nav_2d_msgs::msg::Twist2D & velocity);

protected:
  /**
   * @brief Helper method for two common operations for the operating on the global_plan
   *
   * Transforms the global plan (stored in global_plan_) relative to the pose and saves it in
   * transformed_plan and possibly publishes it. Then it takes the last pose and transforms it
   * to match the local costmap's frame
   */
  void prepareGlobalPlan(
    const nav_2d_msgs::msg::Pose2DStamped & pose, nav_2d_msgs::msg::Path2D & transformed_plan,
    nav_2d_msgs::msg::Pose2DStamped & goal_pose, bool publish_plan = true);

  /**
   * @brief Transforms global plan into same frame as pose, clips far away poses and possibly prunes passed poses
   *
   * Three key operations
   * 1) Transforms global plan into frame of the given pose
   * 2) Only returns poses that are near the robot, i.e. whether they are likely on the local costmap
   * 3) If prune_plan_ is true, it will remove all points that we've already passed from both the transformed plan
   *     and the saved global_plan_. Technically, it iterates to a pose on the path that is within prune_distance_
   *     of the robot and erases all poses before that.
   *
   * Additionally, shorten_transformed_plan_ determines whether we will pass the full plan all
   * the way to the nav goal on to the critics or just a subset of the plan near the robot.
   * True means pass just a subset. This gives DWB less discretion to decide how it gets to the
   * nav goal. Instead it is encouraged to try to get on to the path generated by the global planner.
   */
  virtual nav_2d_msgs::msg::Path2D transformGlobalPlan(
    const nav_2d_msgs::msg::Pose2DStamped & pose);
  nav_2d_msgs::msg::Path2D global_plan_;  ///< Saved Global Plan
  nav_msgs::msg::Path current_path_;
  bool prune_plan_;
  double prune_distance_;
  rclcpp::Duration transform_tolerance_{0, 0};
  bool shorten_transformed_plan_;
  double forward_prune_distance_;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("FollowPosePlanner")};

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::string plugin_name_;




  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_ = nullptr;
  enum class NavState
  {
    NAV_TO_BE_STARTED = 0, // 本次导航待开始，进入computeControl前的初始状态
    NAV_GOING, // 执行正常（非接管）导航中
    NAV_CHECK_FAILED,  // progress check 失败
    NAV_FAILED,  // progress check 失败次数超过阈值，本次规划被terminate
    NAV_SUCCED, // 导航成功
    TAKE_OVER_GOING, // 正在接管中
    TAKE_OVER_SUCCED, // 接管成功
    TAKE_OVER_FAILED, // 接管失败
  };
  NavState nav_state_ = NavState::NAV_TO_BE_STARTED;
  std::mutex mtx_nav_state_;
  int failed_to_make_progress_count_ = 0;

  std::string pub_dynamic_obstacle_markers_topic_name_ = "tros_follow_path_markers";
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      dynamic_obstacle_markers_pub_ = nullptr;
  visualization_msgs::msg::Marker marker_;
  void InitMarker(visualization_msgs::msg::Marker& marker);

  // 有执行Rotate/Move就返回true
  void ClearMarker(std::string ns);
  bool RotateAndMove(float yaw_goal_tolerance, float stop_dist_robot_path_thr);
  bool RotateAndMoveOnce(float yaw_goal_tolerance, float stop_dist_robot_path_thr,
    geometry_msgs::msg::PoseStamped start_robot_pose, nav_2d_msgs::msg::Twist2DStamped& cmd_vel,
    const geometry_msgs::msg::PoseStamped robot_pose);

  int FindPathIndex(const nav_msgs::msg::Path& current_path, float dist_robot_path_thr,
    const geometry_msgs::msg::PoseStamped robot_current_pose);
  // 获取robot最新的pose，计算robot逆时针旋转到输入的path_pose的弧度
  // 成功返回true，以及计算出来的dyaw
  // 失败返回false
  bool GetYawDiff(const geometry_msgs::msg::PoseStamped& path_pose, std::string path_frame_id, float& dyaw,
    const geometry_msgs::msg::PoseStamped robot_current_pose);
  bool IsGlobalPathUpdated();
  bool CheckPathValid(const nav_msgs::msg::Path& path, int path_index);
  void VisualizeGlobalPath();
  int last_recved_global_path_sec_;
  std::string global_path_topic_ = "/transformed_global_plan";
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  void GlobalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  nav_msgs::msg::Path recved_global_path_;
  std::shared_ptr<std::thread> sp_take_over_thread_ = nullptr;
  std::mutex take_over_mutex_;
  std::set<std::string> marker_names_{
    "CUBE", "TakeOver"
  }; 

  // 本次规划器开始时的robot pose
  // 满足以下任意一个条件，就刷新pose：
  // 1. 全局规划的end pose发生变化
  // 2. 距离上次执行computeVelocityCommands的时间超过阈值
  geometry_msgs::msg::PoseStamped start_robot_pose_;
  rclcpp::Time last_compute_cmd_time_;
  bool path_end_pose_updated_ = false;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> sp_last_path_end_pose_ = nullptr;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr follow_pose_start_pose_pub_ = nullptr;

  float rotate_z = 0.8;
};

}  // namespace follow_pose_core

#endif  // FOLLOW_POSE_CORE__FOLLOW_POSE_PLANNER_HPP_

// Copyright (c) 2019 Intel Corporation
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

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_controller/controller_server.hpp"
#include "angles/angles.h"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("controller_server", "", options),
  progress_checker_loader_("nav2_core", "nav2_core::ProgressChecker"),
  default_progress_checker_id_{"progress_checker"},
  default_progress_checker_type_{"nav2_controller::SimpleProgressChecker"},
  goal_checker_loader_("nav2_core", "nav2_core::GoalChecker"),
  default_goal_checker_ids_{"goal_checker"},
  default_goal_checker_types_{"nav2_controller::SimpleGoalChecker"},
  lp_loader_("nav2_core", "nav2_core::Controller"),
  default_ids_{"FollowPath"},
  default_types_{"dwb_core::DWBLocalPlanner"}
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 20.0);

  declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", default_ids_);
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));

  // The costmap node is used in the implementation of the controller
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap");
    
  dynamic_obstacle_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    pub_dynamic_obstacle_markers_topic_name_,
    rclcpp::QoS(10));
    
  // https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html
  marker_.ns = "CUBE";
  marker_.id = 0;
  // Set the marker type.
  marker_.type = visualization_msgs::msg::Marker::CUBE;
  // Set the marker action.  Options are ADD DELETE DELETEALL
  marker_.action = visualization_msgs::msg::Marker::ADD;

  // Marker group position and orientation
  marker_.pose.position.x = 0;
  marker_.pose.position.y = 0;
  marker_.pose.position.z = 0;
  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;
  
  marker_.color.r = 0.0;
  marker_.color.g = 1.0;
  marker_.color.b = 0.0;
  marker_.color.a = 1.0;

  marker_.scale.x = 0.1;
  marker_.scale.y = 0.1;
  marker_.scale.z = 0.01;

  marker_.lifetime = builtin_interfaces::msg::Duration();  // 0 - unlimited
  // marker_.lifetime.sec = 5.0;
  // marker_.lifetime.nanosec = 0.0;
  
  for (const auto& marker_name : marker_names_) {
    ClearMarker(marker_name);
  }
}

void ControllerServer::ClearMarker(std::string ns) {
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

ControllerServer::~ControllerServer()
{
  progress_checker_.reset();
  goal_checkers_.clear();
  controllers_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
ControllerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  get_parameter("progress_checker_plugin", progress_checker_id_);
  if (progress_checker_id_ == default_progress_checker_id_) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_progress_checker_id_ + ".plugin",
      rclcpp::ParameterValue(default_progress_checker_type_));
  }

  RCLCPP_INFO(get_logger(), "getting goal checker plugins..");
  get_parameter("goal_checker_plugins", goal_checker_ids_);
  if (goal_checker_ids_ == default_goal_checker_ids_) {
    for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_goal_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_goal_checker_types_[i]));
    }
  }

  get_parameter("controller_plugins", controller_ids_);
  if (controller_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  controller_types_.resize(controller_ids_.size());
  goal_checker_types_.resize(goal_checker_ids_.size());

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  get_parameter("failure_tolerance", failure_tolerance_);

  costmap_ros_->configure();
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  try {
    progress_checker_type_ = nav2_util::get_plugin_type_param(node, progress_checker_id_);
    progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
    RCLCPP_INFO(
      get_logger(), "Created progress_checker : %s of type %s",
      progress_checker_id_.c_str(), progress_checker_type_.c_str());
    progress_checker_->initialize(node, progress_checker_id_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create progress_checker. Exception: %s", ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    try {
      goal_checker_types_[i] = nav2_util::get_plugin_type_param(node, goal_checker_ids_[i]);
      nav2_core::GoalChecker::Ptr goal_checker =
        goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created goal checker : %s of type %s",
        goal_checker_ids_[i].c_str(), goal_checker_types_[i].c_str());
      goal_checker->initialize(node, goal_checker_ids_[i], costmap_ros_);
      goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create goal checker. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s goal checkers available.", goal_checker_ids_concat_.c_str());

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    try {
      controller_types_[i] = nav2_util::get_plugin_type_param(node, controller_ids_[i]);
      nav2_core::Controller::Ptr controller =
        lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created controller : %s of type %s",
        controller_ids_[i].c_str(), controller_types_[i].c_str());
      controller->configure(
        node, controller_ids_[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create controller. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s controllers available.", controller_ids_concat_.c_str());

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // Create the action server that we implement with our followPath method
  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(),
    "follow_path",
    std::bind(&ControllerServer::computeControl, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  // Set subscribtion to the speed limiting topic
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10),
    std::bind(&ControllerServer::speedLimitCallback, this, std::placeholders::_1));

  // global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
  //   global_path_topic_, rclcpp::QoS(10),
  //   std::bind(&ControllerServer::GlobalPathCallback, this, std::placeholders::_1));
    
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  return nav2_util::CallbackReturn::SUCCESS;
}

void ControllerServer::GlobalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Received global path, ts: %d.%d",
    msg->header.stamp.sec, msg->header.stamp.nanosec);

  // std::lock_guard<std::mutex> lock(global_path_mutex_);
  // recved_global_path_ = *msg;
  // last_recved_global_path_sec_ = recved_global_path_.header.stamp.sec;
}


nav2_util::CallbackReturn
ControllerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  costmap_ros_->activate();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->activate();
  }
  vel_publisher_->on_activate();
  action_server_->activate();

  auto node = shared_from_this();
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ControllerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // 等待接管线程退出
  {
    std::unique_lock<std::mutex> lock(take_over_mutex_);
    if (sp_take_over_thread_ && sp_take_over_thread_->joinable()) {
      RCLCPP_WARN(get_logger(), "waiting for take_over task to finish");
      sp_take_over_thread_->join();
      sp_take_over_thread_.reset();
      sp_take_over_thread_ = nullptr;
      RCLCPP_WARN(get_logger(), "take_over task is exited");
    }
  }

  action_server_->deactivate();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->deactivate();
  }

  /*
   * The costmap is also a lifecycle node, so it may have already fired on_deactivate
   * via rcl preshutdown cb. Despite the rclcpp docs saying on_shutdown callbacks fire
   * in the order added, the preshutdown callbacks clearly don't per se, due to using an
   * unordered_set iteration. Once this issue is resolved, we can maybe make a stronger
   * ordering assumption: https://github.com/ros2/rclcpp/issues/2096
   */
  costmap_ros_->deactivate();

  publishZeroVelocity();
  vel_publisher_->on_deactivate();
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // 等待接管线程退出
  {
    std::unique_lock<std::mutex> lock(take_over_mutex_);
    if (sp_take_over_thread_ && sp_take_over_thread_->joinable()) {
      RCLCPP_WARN(get_logger(), "waiting for take_over task to finish");
      sp_take_over_thread_->join();
      sp_take_over_thread_.reset();
      sp_take_over_thread_ = nullptr;
      RCLCPP_WARN(get_logger(), "take_over task is exited");
    }
  }

  // Cleanup the helper classes
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->cleanup();
  }
  controllers_.clear();

  goal_checkers_.clear();

  costmap_ros_->cleanup();


  // Release any allocated resources
  action_server_.reset();
  odom_sub_.reset();
  costmap_thread_.reset();
  vel_publisher_.reset();
  speed_limit_sub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool ControllerServer::findControllerId(
  const std::string & c_name,
  std::string & current_controller)
{
  if (controllers_.find(c_name) == controllers_.end()) {
    if (controllers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No controller was specified in action call."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", controller_ids_concat_.c_str());
      current_controller = controllers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with controller name %s, "
        "which does not exist. Available controllers are: %s.",
        c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
    current_controller = c_name;
  }

  return true;
}

bool ControllerServer::findGoalCheckerId(
  const std::string & c_name,
  std::string & current_goal_checker)
{
  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    if (goal_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No goal checker was specified in parameter 'current_goal_checker'."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", goal_checker_ids_concat_.c_str());
      current_goal_checker = goal_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with goal_checker name %s in parameter"
        " 'current_goal_checker', which does not exist. Available goal checkers are: %s.",
        c_name.c_str(), goal_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected goal checker: %s.", c_name.c_str());
    current_goal_checker = c_name;
  }

  return true;
}

void ControllerServer::computeControl()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");
  auto start_time = this->now();
  try {
    std::string c_name = action_server_->get_current_goal()->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      action_server_->terminate_current();
      return;
    }

    std::string gc_name = action_server_->get_current_goal()->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      action_server_->terminate_current();
      return;
    }

    setPlannerPath(action_server_->get_current_goal()->path);
    progress_checker_->reset();
    failed_to_make_progress_count_ = 0;

    last_valid_cmd_time_ = now();
    rclcpp::WallRate loop_rate(controller_frequency_);
    while (rclcpp::ok()) {
      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
        action_server_->terminate_all();
        publishZeroVelocity();
        return;
      }

      // Don't compute a trajectory until costmap is valid (after clear costmap)
      rclcpp::Rate r(100);
      while (!costmap_ros_->isCurrent()) {
        r.sleep();
      }

      updateGlobalPath();
        
      {
        std::unique_lock<std::mutex> lock(mtx_nav_state_);
        if (NavState::TAKE_OVER_GOING == nav_state_) {
          // 正在接管中
          continue;
        } else if (NavState::NAV_CHECK_FAILED == nav_state_) {
          // 创建接管任务
          std::unique_lock<std::mutex> lock(take_over_mutex_);
          RCLCPP_WARN(get_logger(), "create take_over task");
          if (sp_take_over_thread_ && sp_take_over_thread_->joinable()) {
            // 等待上次接管完成
            RCLCPP_WARN(get_logger(), "waiting for take_over task to finish");
            sp_take_over_thread_->join();
            sp_take_over_thread_.reset();
            sp_take_over_thread_ = nullptr;
            RCLCPP_WARN(get_logger(), "take_over task is exited");
          }

          // 进入接管状态
          nav_state_ = NavState::TAKE_OVER_GOING;
          
          sp_take_over_thread_ = std::make_shared<std::thread>([this]() {
            RCLCPP_WARN(get_logger(), "take_over task start");
            bool take_over_success = RotateAndMove(3.1415 / 180.0 * 10.0, 0.3);
            ClearMarker("TakeOver");
            RCLCPP_WARN(get_logger(), "take_over task done");
            
            {
              std::unique_lock<std::mutex> lock(mtx_nav_state_);
              if (nav_state_ == NavState::TAKE_OVER_GOING) {
                if (take_over_success) {
                  nav_state_ = NavState::TAKE_OVER_SUCCED;
                } else {
                  nav_state_ = NavState::TAKE_OVER_FAILED;
                }
              }
            }
          });
          continue;
        }
      }
      
      // VisualizeGlobalPath();
      {
        std::unique_lock<std::mutex> lock(mtx_nav_state_);
        // 常规导航状态
        nav_state_ = NavState::NAV_GOING;
      }
      computeAndPublishVelocity();

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
          get_logger(), "Control loop missed its desired rate of %.4fHz",
          controller_frequency_);
      }
    }
  } catch (nav2_core::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    RCLCPP_WARN(get_logger(), "action_server_ terminate_current");
    action_server_->terminate_current();
    
    RCLCPP_WARN(get_logger(), "computeControl done, time cost: %.2f sec",
      (this->now() - start_time).seconds());
    {
      std::unique_lock<std::mutex> lock(mtx_nav_state_);
      nav_state_ = NavState::NAV_FAILED;
    }
    return;
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    RCLCPP_WARN(get_logger(), "action_server_ terminate_current with result");
    action_server_->terminate_current(result);
    RCLCPP_WARN(get_logger(), "computeControl done, time cost: %.2f sec",
      (this->now() - start_time).seconds());
    {
      std::unique_lock<std::mutex> lock(mtx_nav_state_);
      nav_state_ = NavState::NAV_FAILED;
    }
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  publishZeroVelocity();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  action_server_->succeeded_current();
  
  {
    std::unique_lock<std::mutex> lock(mtx_nav_state_);
    nav_state_ = NavState::NAV_SUCCED;
  }

  RCLCPP_WARN(get_logger(), "computeControl done, time cost: %.2f sec",
    (this->now() - start_time).seconds());

}

void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
  RCLCPP_DEBUG(
    get_logger(),
    "Providing path to the controller %s", current_controller_.c_str());
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Invalid path, Path is empty.");
  }
  controllers_[current_controller_]->setPlan(path);

  end_pose_ = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;
  goal_checkers_[current_goal_checker_]->reset();

  RCLCPP_WARN(
    get_logger(), "Path frame_id: %s, end point is (%.2f, %.2f)",
    path.header.frame_id.data(),
    end_pose_.pose.position.x, end_pose_.pose.position.y);

  current_path_ = path;
}

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

bool ControllerServer::IsGlobalPathUpdated() {

  updateGlobalPath();
  if (last_recved_global_path_sec_ == current_path_.header.stamp.sec) {
    return false;
  } else {
    last_recved_global_path_sec_ = current_path_.header.stamp.sec;
    return true;
  }

  // std::lock_guard<std::mutex> lock(global_path_mutex_);
  // if (last_recved_global_path_sec_ == recved_global_path_.header.stamp.sec) {
  //   return false;
  // } else {
  //   last_recved_global_path_sec_ = recved_global_path_.header.stamp.sec;
  //   return true;
  // }

  return false;
}

// 根据规划的路径current_path，寻找第一个和robot距离超过dist_robot_path_thr的规划点
// 返回对应的规划点的索引idx
int ControllerServer::FindPathIndex(const nav_msgs::msg::Path& current_path,
  float dist_robot_path_thr) {
  geometry_msgs::msg::PoseStamped robot_current_pose;
  if (!getRobotPose(robot_current_pose)) {
    return -1;
  }
  if (current_path.poses.empty()) {
    RCLCPP_WARN(get_logger(), "Path is empty");
    return -1;
  }

  last_recved_global_path_sec_ = current_path.header.stamp.sec;

  RCLCPP_INFO(get_logger(), "robot pose ts: %d.%d, current_path ts: %d.%d",
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
        this->get_logger(),
        "Could not transform %s to %s: %s",
        robot_frame.c_str(), path_frame.c_str(), ex.what());
      return -1;
    }
  }

  // 先找到和robot距离最近的点，作为起点
  // 避免因为robot移动了一段距离后，而plan未更新，导致robot返回起点
  int dist_min_path_index = 0;
  float dist_min = 0;
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
    RCLCPP_ERROR(get_logger(), "Find the path point fail, dist_robot_path: %.2f, dist_robot_path_thr: %.2f",
      dist_robot_path, dist_robot_path_thr);
    return -1;
  }

  RCLCPP_WARN(get_logger(), "Find the path point success, robot frame: %s, (%.2f, %.2f), dist_min_path_index: %d, dist_min: %.2f, path frame: %s, (%.2f, %.2f), index: %d, dist_robot_path: %.2f, dist_robot_path_thr: %.2f",
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

bool ControllerServer::GetYawDiff(
  const geometry_msgs::msg::PoseStamped& path_pose,
  std::string path_frame, float& dyaw) {
  geometry_msgs::msg::PoseStamped robot_current_pose;
  if (!getRobotPose(robot_current_pose)) {
    RCLCPP_ERROR(get_logger(), "get robot pose failed");
    return false;
  }

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
    RCLCPP_ERROR(this->get_logger(), "Can't transform %s or %s to %s: %s",
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

  RCLCPP_INFO(get_logger(), "dyaw: %.2f, %d, robot at frame: %s, (%.2f, %.2f); frame: %s, (%.2f, %.2f), forward (%.2f, %.2f), path (%.2f, %.2f); path at frame: %s, (%.2f, %.2f)",
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

bool ControllerServer::CheckPathValid(const nav_msgs::msg::Path& path, int path_index) {
  nav2_costmap_2d::Costmap2D * costmap_ = costmap_ros_->getCostmap();
  if (!costmap_) return false;
  unsigned int cell_x, cell_y;
  for (size_t idx = 0; idx < path.poses.size(); idx++) {
    const auto& pose = path.poses.at(idx);
    if (path_index > 0 && static_cast<int>(idx) >= path_index) {
      break;
    }

    if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, cell_x, cell_y)) {
      RCLCPP_WARN(get_logger(), "Gloabl plan at (%.2f, %.2f) is outside of grid.",
        pose.pose.position.x, pose.pose.position.y);
      continue;
    }
    unsigned char cost = costmap_->getCost(cell_x, cell_y);
    // TODO
    // 卡个阈值？
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE
    //  || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE
     ) {
      RCLCPP_WARN(get_logger(), "Global plan at (%.2f, %.2f) is in an obstacle, cost %d.",
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

void ControllerServer::VisualizeGlobalPath() {
  // 最新的路径规划
  nav_msgs::msg::Path current_path;
  {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    // current_path = recved_global_path_;
    current_path = current_path_;
  }
  if (current_path.poses.empty()) {
    return;
  }
  
  std::string path_frame = current_path.header.frame_id;
  std::string base_link_frame_id = "base_link";
  geometry_msgs::msg::Transform transform_path2baselink =
    tf_buffer_->lookupTransform(base_link_frame_id, path_frame,
    current_path.header.stamp, tf2::durationFromSec(0.1)).transform;
  tf2::Transform tf2_transform;
  tf2::fromMsg(transform_path2baselink, tf2_transform);
  
  auto path_pose = current_path.poses.at(current_path.poses.size() / 2);
  double path_x = 0;
  double path_y = 0;
  {
    tf2::Vector3 p(path_pose.pose.position.x, path_pose.pose.position.y, 0);
    p = tf2_transform * p;
    path_x = p.x();
    path_y = p.y();
  }

  if (dynamic_obstacle_markers_pub_)
  {
    // 发布找到的规划点
    visualization_msgs::msg::MarkerArray marker_array;
    int count = 0;
    auto marker = marker_;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.01;
    marker.header = current_path.header;
    marker.ns = "VisualizeGlobalPath";
    marker.action = visualization_msgs::msg::Marker::ADD;
    {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.id = count++;
      marker.pose.position.x = path_pose.pose.position.x;
      marker.pose.position.y = path_pose.pose.position.y;
      marker_array.markers.push_back(marker);
    }
    {
      marker.header.frame_id = base_link_frame_id;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
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
}

// 
bool ControllerServer::RotateAndMove(float yaw_goal_tolerance, float stop_dist_thr) {
  // 最新的路径规划
  nav_msgs::msg::Path current_path;
  {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    // current_path = recved_global_path_;
    current_path = current_path_;
  }

  // 寻找和robot距离为 dist_robot_path_thr 的最近路径规划点
  float dist_robot_path_thr = stop_dist_thr;
  int path_index = FindPathIndex(current_path, dist_robot_path_thr);
  if (path_index < 0) {
    RCLCPP_ERROR(get_logger(), "find path index fail");
    return false;
  }

  float rotate_z = 0.8;
  geometry_msgs::msg::TwistStamped cmd_vel_2d;
  cmd_vel_2d.twist.angular.x = 0;
  cmd_vel_2d.twist.angular.y = 0;
  cmd_vel_2d.twist.angular.z = 0;
  cmd_vel_2d.twist.linear.x = 0;
  cmd_vel_2d.twist.linear.y = 0;
  cmd_vel_2d.twist.linear.z = 0;
  cmd_vel_2d.header = current_path.header;

  auto get_distance = [this](geometry_msgs::msg::PoseStamped dest_pose)->float{
    geometry_msgs::msg::PoseStamped robot_current_pose;
    if (!getRobotPose(robot_current_pose)) {
      RCLCPP_ERROR(get_logger(), "get robot pose failed");
      return -1;
    }

    // 计算robot和path之间的距离
    float dist = std::hypot(robot_current_pose.pose.position.x - dest_pose.pose.position.x,
                  robot_current_pose.pose.position.y - dest_pose.pose.position.y);
    RCLCPP_INFO(get_logger(), "robot pose frame_id: %s, (%.2f, %.2f), ts: %d.%d, dest (%.2f, %.2f), dist: %.2f",
      robot_current_pose.header.frame_id.data(),
      robot_current_pose.pose.position.x, robot_current_pose.pose.position.y,
      robot_current_pose.header.stamp.sec, robot_current_pose.header.stamp.nanosec,
      dest_pose.pose.position.x, dest_pose.pose.position.y,
      dist
      );
      
    return dist;
  };
    
  // 开始控制的时间，用于超时时间的计算
  auto time_start = this->now();
  // 开始平移时，robot的起点，用于控制平移完成
  std::shared_ptr<geometry_msgs::msg::PoseStamped> sp_start_robot_pose = nullptr;
  while (rclcpp::ok()) {
    if (this->now() - time_start > std::chrono::seconds(5)) {
      RCLCPP_WARN(this->get_logger(), "move timeout: %f sec", (this->now() - time_start).seconds());
      publishZeroVelocity();
      break;
    }
    
    {
      std::unique_lock<std::mutex> lock(mtx_nav_state_);
      if (nav_state_ != NavState::TAKE_OVER_GOING) {
        RCLCPP_WARN(this->get_logger(), "nav state changed to %d, stop take-over task", static_cast<int>(nav_state_));
        // 状态已经发生变化，停止接管
        publishZeroVelocity();
        break;
      }
    }

    // 检查是否有新的 path
    // if (IsGlobalPathUpdated()) 
    // 不检查更新，避免线程安全问题
    if (0)
    {
      nav_msgs::msg::Path old_path;

      {
        std::lock_guard<std::mutex> lock(global_path_mutex_);
        // current_path = recved_global_path_;
        old_path = current_path;
        current_path = current_path_;
        cmd_vel_2d.header = current_path.header;
      }
      path_index = FindPathIndex(current_path, dist_robot_path_thr);
      if (path_index < 0) {
        RCLCPP_ERROR(get_logger(), "find path index fail");
        return false;
      }

      // TODO 判断path是否是有效更新
      if (!old_path.poses.empty() && !current_path.poses.empty()) {
        RCLCPP_WARN(get_logger(), "global path updated, first pose in new path: (%.2f, %.2f), index: %d, first pose in old path: (%.2f, %.2f)",
          current_path.poses.at(0).pose.position.x, current_path.poses.at(0).pose.position.y,
          path_index,
          old_path.poses.at(0).pose.position.x, old_path.poses.at(0).pose.position.y
          );
      }
    }

    float dyaw;
    if (!GetYawDiff(current_path.poses.at(path_index), current_path.header.frame_id, dyaw)) {
      return false;
    }
    // 将逆时针弧度转成最小弧度夹角
    float yaw_absolute = dyaw;
    if (dyaw > 3.14159) {
      yaw_absolute = 2 * 3.14159 - dyaw;
    }
    // 如果robot和目标位置距离很近，不需要再rotate
    float dist_robot_to_path = get_distance(current_path.poses.at(path_index));
    RCLCPP_INFO(get_logger(), "yaw_absolute: %.2f, %d, yaw_goal_tolerance: %.2f, %d, dist_robot_to_path: %.2f",
      yaw_absolute, static_cast<int>(yaw_absolute * 180.0 / 3.14159),
      yaw_goal_tolerance, static_cast<int>(yaw_goal_tolerance * 180.0 / 3.14159),
      dist_robot_to_path);
    if (yaw_absolute < yaw_goal_tolerance || dist_robot_to_path < 0.1) {
      RCLCPP_INFO(get_logger(), "do not need to rotate");
    } else {
      cmd_vel_2d.twist.linear.x = 0;
      cmd_vel_2d.twist.linear.y = 0;
      cmd_vel_2d.twist.linear.z = 0;
      cmd_vel_2d.twist.angular.x = 0.0;
      cmd_vel_2d.twist.angular.y = 0;
      if (dyaw < 3.14159) {
        // 逆时针旋转
        cmd_vel_2d.twist.angular.z = (1.0) * rotate_z;
        RCLCPP_INFO(get_logger(), "rotate anti-clockwise");
      } else {
        // 顺时针旋转
        cmd_vel_2d.twist.angular.z = (-1.0) * rotate_z;
        RCLCPP_INFO(get_logger(), "rotate clockwise");
      }

      // 如果角度很小，减小旋转速度
      if (yaw_absolute < yaw_goal_tolerance * 2.0) {
        // 转动了超过一半
        cmd_vel_2d.twist.angular.z = cmd_vel_2d.twist.angular.z * 0.5;
      }

      publishVelocity(cmd_vel_2d);
      
      // std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
      // feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);
      // action_server_->publish_feedback(feedback);

      rclcpp::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    // 运行到这里说明角度小于阈值

    // 移动
    // 计算每个规划点对应cell的cost，判断是否为有效路径
    if (!CheckPathValid(current_path, path_index * 2)) {
      RCLCPP_ERROR(get_logger(), "path is not valid, which is in obstacle");
      return false;
    }
    if (!sp_start_robot_pose) {
      // 设置平移起点
      sp_start_robot_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
      geometry_msgs::msg::PoseStamped robot_current_pose;
      if (getRobotPose(robot_current_pose)) {
        *sp_start_robot_pose = robot_current_pose;
      } else {
        break;
      }
    }

    // 计算robot当前位置和移动前位置之间的距离
    float dist_robot_to_start = get_distance(*sp_start_robot_pose);
    if (dist_robot_to_start < 0) {
      // 计算失败
      return false;
    }
    RCLCPP_INFO(get_logger(),
      "dist_robot_to_start: %.2f, stop_dist_thr: %.2f",
      dist_robot_to_start, stop_dist_thr);

    if (dist_robot_to_start >= stop_dist_thr) {
      // 移动了足够远的距离，停止
      RCLCPP_INFO(get_logger(), "no need to move");
      // 不需要旋转和平移，继续跑轨迹规划
      break;
    } else {
      RCLCPP_INFO(get_logger(), "do move");
      cmd_vel_2d.twist.linear.x = 0.2;
      cmd_vel_2d.twist.linear.y = 0;
      cmd_vel_2d.twist.linear.z = 0;
      cmd_vel_2d.twist.angular.x = 0.0;
      cmd_vel_2d.twist.angular.y = 0;
      cmd_vel_2d.twist.angular.z = 0;
      if (dist_robot_to_start >= stop_dist_thr * 0.5) {
        // 移动了超过一半
        cmd_vel_2d.twist.linear.x = cmd_vel_2d.twist.linear.x * 0.5;
      }

      publishVelocity(cmd_vel_2d);

      // std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
      // feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);
      // action_server_->publish_feedback(feedback);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
  }

  float dyaw;
  if (!GetYawDiff(current_path.poses.at(path_index), current_path.header.frame_id, dyaw)) {
    return false;
  }
  float dist_robot_to_start = -1;
  if (sp_start_robot_pose) {
    dist_robot_to_start = get_distance(*sp_start_robot_pose);
  }
  RCLCPP_WARN(get_logger(), "after rotate and move, dyaw: %.2f, %d, dist_robot_to_start: %.2f",
    dyaw, static_cast<int>(dyaw * 180.0 / 3.14159), dist_robot_to_start);
  publishZeroVelocity();

  return true;
}

void ControllerServer::computeAndPublishVelocity()
{
  RCLCPP_INFO(get_logger(), "do compute And PublishVelocity");
  geometry_msgs::msg::PoseStamped pose;
  
  if (!getRobotPose(pose)) {
    throw nav2_core::PlannerException("Failed to obtain robot pose");
  }

  if (!progress_checker_->check(pose)) {
    // 不抛异常
    // 此处抛异常会导致重新发起导航action请求
    // throw nav2_core::PlannerException("Failed to make progress");
    failed_to_make_progress_count_++;
    RCLCPP_ERROR(get_logger(), "Failed to make progress, failed_to_make_progress_count_: %d",
      failed_to_make_progress_count_);
    if (failed_to_make_progress_count_ >= 3) {
      throw nav2_core::PlannerException("Failed to make progress");
    } else {
      std::unique_lock<std::mutex> lock(mtx_nav_state_);
      nav_state_ = NavState::NAV_CHECK_FAILED;
    }
    return;
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());

  geometry_msgs::msg::TwistStamped cmd_vel_2d;

  try {
    cmd_vel_2d =
      controllers_[current_controller_]->computeVelocityCommands(
      pose,
      nav_2d_utils::twist2Dto3D(twist),
      goal_checkers_[current_goal_checker_].get());
    last_valid_cmd_time_ = now();
  } catch (nav2_core::PlannerException & e) {
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      cmd_vel_2d.twist.angular.x = 0;
      cmd_vel_2d.twist.angular.y = 0;
      cmd_vel_2d.twist.angular.z = 0;
      cmd_vel_2d.twist.linear.x = 0;
      cmd_vel_2d.twist.linear.y = 0;
      cmd_vel_2d.twist.linear.z = 0;
      cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
      cmd_vel_2d.header.stamp = now();
      if ((now() - last_valid_cmd_time_).seconds() > failure_tolerance_ &&
        failure_tolerance_ != -1.0)
      {
        throw nav2_core::PlannerException("Controller patience exceeded");
      }
    } else {
      throw nav2_core::PlannerException(e.what());
    }
  }

  std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

  // Find the closest pose to current pose on global path
  nav_msgs::msg::Path & current_path = current_path_;
  auto find_closest_pose_idx =
    [&pose, &current_path]() {
      size_t closest_pose_idx = 0;
      double curr_min_dist = std::numeric_limits<double>::max();
      for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
        double curr_dist = nav2_util::geometry_utils::euclidean_distance(
          pose, current_path.poses[curr_idx]);
        if (curr_dist < curr_min_dist) {
          curr_min_dist = curr_dist;
          closest_pose_idx = curr_idx;
        }
      }
      return closest_pose_idx;
    };

  feedback->distance_to_goal =
    nav2_util::geometry_utils::calculate_path_length(current_path_, find_closest_pose_idx());
  action_server_->publish_feedback(feedback);

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);
}

void ControllerServer::updateGlobalPath()
{
  if (!action_server_) {
    RCLCPP_ERROR(get_logger(), "Action server is invalid");
    return;
  }

  if (action_server_->is_preempt_requested()) {
    RCLCPP_INFO(get_logger(), "Passing new path to controller.");
    auto goal = action_server_->accept_pending_goal();
    std::string current_controller;
    if (findControllerId(goal->controller_id, current_controller)) {
      current_controller_ = current_controller;
    } else {
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid controller %s requested.",
        goal->controller_id.c_str());
      action_server_->terminate_current();
      return;
    }
    std::string current_goal_checker;
    if (findGoalCheckerId(goal->goal_checker_id, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid goal checker %s requested.",
        goal->goal_checker_id.c_str());
      action_server_->terminate_current();
      return;
    }
    setPlannerPath(goal->path);
  }
}

void ControllerServer::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (vel_publisher_->is_activated() && vel_publisher_->get_subscription_count() > 0) {
    
    RCLCPP_DEBUG(get_logger(), "publish Velocity x: %.2f, z: %.2f", cmd_vel->linear.x, cmd_vel->angular.z);
    vel_publisher_->publish(std::move(cmd_vel));
  }
}

void ControllerServer::publishZeroVelocity()
{
  RCLCPP_WARN(this->get_logger(), "publish Zero Velocity");
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  velocity.header.frame_id = costmap_ros_->getBaseFrameID();
  velocity.header.stamp = now();
  publishVelocity(velocity);
}

bool ControllerServer::isGoalReached()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    return false;
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());
  geometry_msgs::msg::Twist velocity = nav_2d_utils::twist2Dto3D(twist);

  geometry_msgs::msg::PoseStamped transformed_end_pose;
  rclcpp::Duration tolerance(rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
  nav_2d_utils::transformPose(
    costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
    end_pose_, transformed_end_pose, tolerance);


  // RCLCPP_INFO_STREAM(rclcpp::get_logger("ControllerServer"),
  // "check isGoalReached"
  // << ", costmap_ros_->getGlobalFrameID: " << costmap_ros_->getGlobalFrameID()
  // << ", end_pose_ frame_id: " << end_pose_.header.frame_id
  // << ", x: " << end_pose_.pose.position.x << ", y: " << end_pose_.pose.position.y
  // << ", transformed_end_pose frame_id: " << transformed_end_pose.header.frame_id
  // << ", x: " << transformed_end_pose.pose.position.x << ", y: " << transformed_end_pose.pose.position.y
  // );

  return goal_checkers_[current_goal_checker_]->isGoalReached(
    pose.pose, transformed_end_pose.pose,
    velocity);
}

bool ControllerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    return false;
  }
  pose = current_pose;
  return true;
}

void ControllerServer::speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg)
{
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
}

rcl_interfaces::msg::SetParametersResult
ControllerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // If we are trying to change the parameter of a plugin we can just skip it at this point
    // as they handle parameter changes themselves and don't need to lock the mutex
    if (name.find('.') != std::string::npos) {
      continue;
    }

    if (!dynamic_params_lock_.try_lock()) {
      RCLCPP_WARN(
        get_logger(),
        "Unable to dynamically change Parameters while the controller is currently running");
      result.successful = false;
      result.reason =
        "Unable to dynamically change Parameters while the controller is currently running";
      return result;
    }

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "min_x_velocity_threshold") {
        min_x_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_y_velocity_threshold") {
        min_y_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_theta_velocity_threshold") {
        min_theta_velocity_threshold_ = parameter.as_double();
      } else if (name == "failure_tolerance") {
        failure_tolerance_ = parameter.as_double();
      }
    }

    dynamic_params_lock_.unlock();
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_controller::ControllerServer)

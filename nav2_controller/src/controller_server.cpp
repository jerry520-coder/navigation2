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
  // 获取当前节点的共享指针。
  auto node = shared_from_this();

  // 使用RCLCPP_INFO记录日志信息。
  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  // 从参数服务器获取名为"progress_checker_plugin"的参数，并存储在progress_checker_id_变量中。
  get_parameter("progress_checker_plugin", progress_checker_id_);
  // 如果progress_checker_id_的值等于默认值default_progress_checker_id_，则声明一个新的参数。
  if (progress_checker_id_ == default_progress_checker_id_) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_progress_checker_id_ + ".plugin",
      rclcpp::ParameterValue(default_progress_checker_type_));
  }

  // 获取名为"goal_checker_plugins"的参数，并存储在goal_checker_ids_变量中。
  // 如果goal_checker_ids_的值等于默认值default_goal_checker_ids_，则为每个默认的goal checker声明一个新的参数。
  RCLCPP_INFO(get_logger(), "getting goal checker plugins..");
  get_parameter("goal_checker_plugins", goal_checker_ids_);
  if (goal_checker_ids_ == default_goal_checker_ids_) {
    for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_goal_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_goal_checker_types_[i]));
    }
  }

  // 获取名为"controller_plugins"的参数，并存储在controller_ids_变量中。
  // 如果controller_ids_的值等于默认值default_ids_，则为每个默认的controller声明一个新的参数。
  get_parameter("controller_plugins", controller_ids_);
  if (controller_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  // 初始化存储controller和goal checker类型的容器。
  controller_types_.resize(controller_ids_.size());
  goal_checker_types_.resize(goal_checker_ids_.size());

  // 获取其他一些参数。
  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  // 获取速度限制话题的名称和容错参数。
  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  get_parameter("failure_tolerance", failure_tolerance_);

  // 配置costmap节点。
  costmap_ros_->configure();
  // 启动一个线程来运行costmap节点。
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  // 尝试创建进度检查器插件。
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

  // 为每个goal checker插件创建一个实例，并初始化它们。
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

  // 将goal checker的ID连接成一个字符串，用于日志信息。
  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }

  // 记录当前可用的goal checker。
  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s goal checkers available.", goal_checker_ids_concat_.c_str());

  // 为每个controller插件创建一个实例，并配置它们。
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

  // 将controller的ID连接成一个字符串，用于日志信息。
  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  // 记录当前可用的controller。
  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s controllers available.", controller_ids_concat_.c_str());

  // 创建一个订阅器来接收里程计信息。
  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  // 创建一个发布器来发布cmd_vel消息。
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // 创建一个行为服务器，用于实现路径跟随功能。
  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(),
    "follow_path",
    std::bind(&ControllerServer::computeControl, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  // 创建一个订阅器来接收速度限制信息。
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10),
    std::bind(&ControllerServer::speedLimitCallback, this, std::placeholders::_1));

  // 配置成功，返回SUCCESS。
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating"); // 记录信息，表示控制器服务器正在激活

  costmap_ros_->activate(); // 激活代价地图
  ControllerMap::iterator it; // 定义迭代器用于遍历控制器映射
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->activate(); // 遍历并激活所有控制器
  }
  vel_publisher_->on_activate(); // 激活速度发布器
  action_server_->activate(); // 激活动作服务器

  auto node = shared_from_this(); // 获取当前节点的共享指针
  // 添加动态参数的回调
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ControllerServer::dynamicParametersCallback, this, _1));

  // 创建bond连接
  createBond();

  return nav2_util::CallbackReturn::SUCCESS; // 返回成功状态
}

nav2_util::CallbackReturn
ControllerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

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
  if (costmap_ros_->get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    costmap_ros_->deactivate();
  }

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

  // Cleanup the helper classes
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->cleanup();
  }
  controllers_.clear();

  goal_checkers_.clear();

  if (costmap_ros_->get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    costmap_ros_->cleanup();
  }

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
  std::lock_guard<std::mutex> lock(dynamic_params_lock_); // 锁定动态参数，确保线程安全

  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort."); // 记录接收到目标的信息

  try {
    // 获取当前目标的控制器ID，并尝试找到对应的控制器
    std::string c_name = action_server_->get_current_goal()->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      action_server_->terminate_current(); // 如果未找到控制器，终止当前目标
      return;
    }

    // 获取当前目标的目标检查器ID，并尝试找到对应的目标检查器
    std::string gc_name = action_server_->get_current_goal()->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      action_server_->terminate_current(); // 如果未找到目标检查器，终止当前目标
      return;
    }

    setPlannerPath(action_server_->get_current_goal()->path); // 设置规划器路径
    progress_checker_->reset(); // 重置进度检查器

    last_valid_cmd_time_ = now(); // 更新上一次有效命令的时间
    rclcpp::WallRate loop_rate(controller_frequency_); // 设置控制循环频率
    while (rclcpp::ok()) {
      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        // 如果动作服务器不可用或不活跃，则停止
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        // 如果请求取消目标，则停止机器人
        RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
        action_server_->terminate_all();
        publishZeroVelocity();
        return;
      }

      // 在代价地图有效之前（例如，清除代价地图之后），不计算轨迹
      rclcpp::Rate r(100);
      while (!costmap_ros_->isCurrent()) {
        r.sleep();
      }

      updateGlobalPath(); // 更新全局路径

      computeAndPublishVelocity(); // 计算并发布速度

      if (isGoalReached()) {
        // 如果达到目标，则退出循环
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      if (!loop_rate.sleep()) {
        // 如果控制循环未能按预期频率执行，记录警告
        RCLCPP_WARN(
          get_logger(), "Control loop missed its desired rate of %.4fHz",
          controller_frequency_);
      }
    }
  } catch (nav2_core::PlannerException & e) {
    // 捕获并处理规划器异常
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    action_server_->terminate_current();
    return;
  } catch (std::exception & e) {
    // 捕获并处理其他异常
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    action_server_->terminate_current(result);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result"); // 控制器成功，设置结果

  publishZeroVelocity(); // 发布零速度，停止机器人

  // TODO: 处理潜在的抢占请求，并设置控制器名称
  action_server_->succeeded_current();
}

void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
  // 在调试日志中记录提供路径给控制器的信息，包括当前控制器的名称
  RCLCPP_DEBUG(
    get_logger(),
    "Providing path to the controller %s", current_controller_.c_str());
  
  // 检查提供的路径是否为空，如果为空，则抛出一个PlannerException异常
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Invalid path, Path is empty.");
  }
  
  // 将路径设置到当前控制器中。这里假设`controllers_`是一个映射，
  // 其中键是控制器的名称，值是控制器的实例
  controllers_[current_controller_]->setPlan(path);

  // 获取路径的最后一个姿态，并将其保存到`end_pose_`成员变量中
  // 同时，确保`end_pose_`的帧ID与路径的帧ID一致
  end_pose_ = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;

  // 重置与当前目标检查器关联的状态。这里假设`goal_checkers_`是一个映射，
  // 其中键是目标检查器的名称，值是目标检查器的实例
  goal_checkers_[current_goal_checker_]->reset();

  // 在调试日志中记录路径终点的位置信息
  RCLCPP_DEBUG(
    get_logger(), "Path end point is (%.2f, %.2f)",
    end_pose_.pose.position.x, end_pose_.pose.position.y);

  // 将提供的路径保存到`current_path_`成员变量中，以便后续使用
  current_path_ = path;
}

void ControllerServer::computeAndPublishVelocity()
{
  geometry_msgs::msg::PoseStamped pose; // 定义一个姿态变量，用于存储机器人当前姿态

  //获取global_frame的当前位姿
  //tf2::Transform::getIdentity() 返回一个单位变换矩阵，表示没有旋转和平移的变换。
  if (!getRobotPose(pose)) { 
    // 尝试获取机器人当前的姿态，如果失败，则抛出异常
    throw nav2_core::PlannerException("Failed to obtain robot pose");
  }

  if (!progress_checker_->check(pose)) {
    // 使用进度检查器检查机器人是否在向目标进展，如果没有，则抛出异常
    throw nav2_core::PlannerException("Failed to make progress");
  }

  // 获取从里程计订阅者获得的调整后的速度（考虑到阈值）
  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());

  geometry_msgs::msg::TwistStamped cmd_vel_2d; // 定义一个速度命令变量

  try {
    // 调用当前控制器的computeVelocityCommands方法计算速度命令
    cmd_vel_2d =
      controllers_[current_controller_]->computeVelocityCommands(
        pose, //获取global_frame的当前位姿
        nav_2d_utils::twist2Dto3D(twist), // 将2D速度转换为3D。里程计获得的速度
        goal_checkers_[current_goal_checker_].get()); // 获取当前目标检查器
    last_valid_cmd_time_ = now(); // 更新最后一次有效命令的时间
  } catch (nav2_core::PlannerException & e) {
    // 捕捉到规划器异常
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      // 如果设置了失败容忍度，则发出警告并发布零速度
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      // 设置零速度
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
        // 如果超过了失败容忍时间，再次抛出异常
        throw nav2_core::PlannerException("Controller patience exceeded");
      }
    } else {
      // 如果没有设置失败容忍度，直接重新抛出异常
      throw nav2_core::PlannerException(e.what());
    }
  }

  // 创建反馈信息，并计算当前速度和到目标的距离
  std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

  // 计算当前位置到全局路径上最近点的距离
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
    nav2_util::geometry_utils::calculate_path_length(current_path_, find_closest_pose_idx()); //到目标的的剩余距离
  action_server_->publish_feedback(feedback); // 发布反馈信息

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds()); // 记录调试信息
  publishVelocity(cmd_vel_2d); // 发布速度命令
}

void ControllerServer::updateGlobalPath()
{
  // 检查动作服务器是否收到了抢占请求
  if (action_server_->is_preempt_requested()) {
    RCLCPP_INFO(get_logger(), "Passing new path to controller."); // 记录信息，表示正在向控制器传递新路径
    
    auto goal = action_server_->accept_pending_goal(); // 接受待处理的目标，这里的目标包含新的路径
    
    std::string current_controller;
    // 查找与目标中指定的控制器ID对应的控制器，如果找到则更新当前控制器
    if (findControllerId(goal->controller_id, current_controller)) {
      current_controller_ = current_controller;
    } else {
      // 如果没有找到对应的控制器，记录信息并终止当前的动作
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid controller %s requested.",
        goal->controller_id.c_str());
      action_server_->terminate_current();
      return;
    }
    
    std::string current_goal_checker;
    // 查找与目标中指定的目标检查器ID对应的目标检查器，如果找到则更新当前目标检查器
    if (findGoalCheckerId(goal->goal_checker_id, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      // 如果没有找到对应的目标检查器，记录信息并终止当前的动作
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid goal checker %s requested.",
        goal->goal_checker_id.c_str());
      action_server_->terminate_current();
      return;
    }
    
    setPlannerPath(goal->path); // 将新路径设置给控制器
  }
}

void ControllerServer::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (vel_publisher_->is_activated() && vel_publisher_->get_subscription_count() > 0) {
    vel_publisher_->publish(std::move(cmd_vel));
  }
}

void ControllerServer::publishZeroVelocity()
{
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
  geometry_msgs::msg::PoseStamped pose; // 定义一个变量用于存储机器人当前的姿态

  if (!getRobotPose(pose)) {
    // 尝试获取机器人当前姿态，如果失败，则返回false表示目标未达到
    return false;
  }

  // 获取经过阈值处理后的当前速度
  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());
  geometry_msgs::msg::Twist velocity = nav_2d_utils::twist2Dto3D(twist); // 将2D速度转换为3D格式

  geometry_msgs::msg::PoseStamped transformed_end_pose; // 定义一个变量用于存储转换后的路径终点姿态
  rclcpp::Duration tolerance(rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
  // 将路径终点姿态从路径终点的坐标系转换到机器人当前坐标系中
  nav_2d_utils::transformPose(
    costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
    end_pose_, transformed_end_pose, tolerance);

  // 调用当前目标检查器的isGoalReached方法来判断机器人是否达到了目标
  // 该方法考虑了机器人的姿态、目标姿态和机器人的速度
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

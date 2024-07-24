// Copyright (c) 2021 Samsung Research America
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
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"

using rcl_interfaces::msg::ParameterType;

namespace nav2_rotation_shim_controller
{

RotationShimController::RotationShimController()
: lp_loader_("nav2_core", "nav2_core::Controller"),
  primary_controller_(nullptr),
  path_updated_(false)
{
}

void RotationShimController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = name;
  node_ = parent;
  auto node = parent.lock();

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  std::string primary_controller;
  double control_frequency;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".angular_dist_threshold", rclcpp::ParameterValue(0.785));  // 45 deg
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".forward_sampling_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".simulate_ahead_time", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".primary_controller", rclcpp::PARAMETER_STRING);

  node->get_parameter(plugin_name_ + ".angular_dist_threshold", angular_dist_threshold_);
  node->get_parameter(plugin_name_ + ".forward_sampling_distance", forward_sampling_distance_);
  node->get_parameter(
    plugin_name_ + ".rotate_to_heading_angular_vel",
    rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".simulate_ahead_time", simulate_ahead_time_);

  primary_controller = node->get_parameter(plugin_name_ + ".primary_controller").as_string();
  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  try {
    primary_controller_ = lp_loader_.createUniqueInstance(primary_controller);
    RCLCPP_INFO(
      logger_, "Created internal controller for rotation shimming: %s of type %s",
      plugin_name_.c_str(), primary_controller.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      logger_,
      "Failed to create internal controller for rotation shimming. Exception: %s", ex.what());
    return;
  }

  primary_controller_->configure(parent, name, tf, costmap_ros);

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_ros->getCostmap());
}

void RotationShimController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "nav2_rotation_shim_controller::RotationShimController",
    plugin_name_.c_str());

  primary_controller_->activate();

  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &RotationShimController::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void RotationShimController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "nav2_rotation_shim_controller::RotationShimController",
    plugin_name_.c_str());

  primary_controller_->deactivate();

  dyn_params_handler_.reset();
}

void RotationShimController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type "
    "nav2_rotation_shim_controller::RotationShimController",
    plugin_name_.c_str());

  primary_controller_->cleanup();
  primary_controller_.reset();
}

geometry_msgs::msg::TwistStamped RotationShimController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (path_updated_) {
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex())); // 锁定成本图，以确保线程安全

    std::lock_guard<std::mutex> lock_reinit(mutex_);
    try {
      geometry_msgs::msg::Pose sampled_pt_base = transformPoseToBaseFrame(getSampledPathPt()); //robot_base_frame_

      // 计算角度距离到新的朝向
      double angular_distance_to_heading =
        std::atan2(sampled_pt_base.position.y, sampled_pt_base.position.x);

        // 如果角度距离大于阈值，则需要旋转
      if (fabs(angular_distance_to_heading) > angular_dist_threshold_) {
        RCLCPP_DEBUG(
          logger_,
          "Robot is not within the new path's rough heading, rotating to heading...");
        return computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
      } else {
        RCLCPP_DEBUG(
          logger_,
          "Robot is at the new path's rough heading, passing to controller"); // 如果已经在新路径的大致朝向上，将控制权交给主控制器
        path_updated_ = false;
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_DEBUG(
        logger_,
        "Rotation Shim Controller was unable to find a sampling point,"
        " a rotational collision was detected, or TF failed to transform"
        " into base frame! what(): %s", e.what());// 如果在找采样点、检测旋转冲突或转换失败时抛出异常
      path_updated_ = false;
    }
  }

  // If at this point, use the primary controller to path track
  return primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);// 如果到了这一步，使用主控制器来进行路径跟踪
}

geometry_msgs::msg::PoseStamped RotationShimController::getSampledPathPt()
{
  if (current_path_.poses.size() < 2) {
    // 如果是，抛出规划异常，说明路径过短，无法找到有效的采样路径点用于旋转
    throw nav2_core::PlannerException(
            "Path is too short to find a valid sampled path point for rotation.");
  }

// 获取路径中的第一个姿态点作为起始点
  geometry_msgs::msg::Pose start = current_path_.poses.front().pose;
  double dx, dy; // 定义变量用于计算两点间的横纵坐标差

  // Find the first point at least sampling distance away
  for (unsigned int i = 1; i != current_path_.poses.size(); i++) { // 循环遍历路径中的每个点，从第二个点开始

   // 计算当前点与起始点的x和y坐标差
    dx = current_path_.poses[i].pose.position.x - start.position.x;
    dy = current_path_.poses[i].pose.position.y - start.position.y;

    // 如果当前点与起始点的距离大于或等于前向采样距离
    if (hypot(dx, dy) >= forward_sampling_distance_) {

      // 将当前路径点的帧ID和时间戳更新为路径的帧ID和当前时间
      current_path_.poses[i].header.frame_id = current_path_.header.frame_id;
      current_path_.poses[i].header.stamp = clock_->now();  // Get current time transformation
      return current_path_.poses[i];
    }
  }

// 如果没有找到至少距离机器人指定距离的采样点，抛出规划异常
  throw nav2_core::PlannerException(
          std::string(
            "Unable to find a sampling point at least %0.2f from the robot,"
            "passing off to primary controller plugin.", forward_sampling_distance_));
}

geometry_msgs::msg::Pose
RotationShimController::transformPoseToBaseFrame(const geometry_msgs::msg::PoseStamped & pt)
{
  geometry_msgs::msg::PoseStamped pt_base;
  if (!nav2_util::transformPoseInTargetFrame(pt, pt_base, *tf_, costmap_ros_->getBaseFrameID())) {
    throw nav2_core::PlannerException("Failed to transform pose to base frame!");
  }
  return pt_base.pose;
}

geometry_msgs::msg::TwistStamped
RotationShimController::computeRotateToHeadingCommand(
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  const double sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0; // 确定旋转方向的符号，正为顺时针，负为逆时针
  const double angular_vel = sign * rotate_to_heading_angular_vel_; // 计算旋转速度，旋转方向由 sign 决定
  const double & dt = control_duration_;// 控制周期时间
  const double min_feasible_angular_speed = velocity.angular.z - max_angular_accel_ * dt; // 计算最小和最大可行的角速度，考虑最大角加速度
  const double max_feasible_angular_speed = velocity.angular.z + max_angular_accel_ * dt;
  cmd_vel.twist.angular.z =
    std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed); // 将计算的旋转速度限制在最小和最大可行角速度之间

  isCollisionFree(cmd_vel, angular_distance_to_heading, pose);
  return cmd_vel;
}

void RotationShimController::isCollisionFree(
  const geometry_msgs::msg::TwistStamped & cmd_vel,
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Simulate rotation ahead by time in control frequency increments
   // 模拟控制周期时间内的旋转，以预判是否会发生碰撞
  double simulated_time = 0.0;
  double initial_yaw = tf2::getYaw(pose.pose.orientation);// 获取当前的偏航角
  double yaw = 0.0;
  double footprint_cost = 0.0;  // 用于记录旋转过程中的成本图代价
  double remaining_rotation_before_thresh =
    fabs(angular_distance_to_heading) - angular_dist_threshold_;  // 计算在达到阈值前还需要旋转的角度

 // 在既定的模拟时间内，进行碰撞检测模拟
  while (simulated_time < simulate_ahead_time_) {
    simulated_time += control_duration_;
    yaw = initial_yaw + cmd_vel.twist.angular.z * simulated_time;// 计算模拟时间后的偏航角

    // Stop simulating past the point it would be passed onto the primary controller
    // 如果模拟的旋转角度达到了传递给主控制器之前的角度阈值，则停止模拟
    if (angles::shortest_angular_distance(yaw, initial_yaw) >= remaining_rotation_before_thresh) {
      break;
    }

    using namespace nav2_costmap_2d;  // NOLINT 引入命名空间，用于访问成本图相关功能

      // 计算当前模拟姿态在成本图上的代价
    footprint_cost = collision_checker_->footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y,
      yaw, costmap_ros_->getRobotFootprint());

   // 如果代价指示未知区域，并且成本图正在追踪未知区域
    if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
      throw std::runtime_error("RotationShimController detected a potential collision ahead!");// 抛出异常，表明可能存在碰撞风险
    }


 // 如果代价指示致命障碍
    if (footprint_cost >= static_cast<double>(LETHAL_OBSTACLE)) {
      throw std::runtime_error("RotationShimController detected collision ahead!");// 抛出异常，表明前方检测到碰撞
    }
  }
}

void RotationShimController::setPlan(const nav_msgs::msg::Path & path)
{
  path_updated_ = true;
  current_path_ = path;
  primary_controller_->setPlan(path);
}

void RotationShimController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  primary_controller_->setSpeedLimit(speed_limit, percentage);
}

rcl_interfaces::msg::SetParametersResult
RotationShimController::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".angular_dist_threshold") {
        angular_dist_threshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".forward_sampling_distance") {
        forward_sampling_distance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_angular_vel") {
        rotate_to_heading_angular_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_angular_accel") {
        max_angular_accel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".simulate_ahead_time") {
        simulate_ahead_time_ = parameter.as_double();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_rotation_shim_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_rotation_shim_controller::RotationShimController,
  nav2_core::Controller)

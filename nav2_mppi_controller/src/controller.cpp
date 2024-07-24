// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <stdint.h>
#include <chrono>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{

  void MPPIController::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    parent_ = parent;
    costmap_ros_ = costmap_ros;
    tf_buffer_ = tf;
    name_ = name;
    parameters_handler_ = std::make_unique<ParametersHandler>(parent);

    auto node = parent_.lock();
    clock_ = node->get_clock();
    last_time_called_ = clock_->now();
    // Get high-level controller parameters
    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(visualize_, "visualize", false);
    getParam(reset_period_, "reset_period", 1.0);

    // Configure composed objects
    optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
    path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
    trajectory_visualizer_.on_configure(
        parent_, name_,
        costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

    RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
  }

  void MPPIController::cleanup()
  {
    optimizer_.shutdown();
    trajectory_visualizer_.on_cleanup();
    parameters_handler_.reset();
    RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
  }

  void MPPIController::activate()
  {
    trajectory_visualizer_.on_activate();
    parameters_handler_->start();
    RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
  }

  void MPPIController::deactivate()
  {
    trajectory_visualizer_.on_deactivate();
    RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
  }

  void MPPIController::reset()
  {
    optimizer_.reset();
  }

  geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &robot_pose, // 机器人当前姿态
      const geometry_msgs::msg::Twist &robot_speed,      // 机器人当前速度
      nav2_core::GoalChecker *goal_checker)              // 目标检查器指针
  {
#ifdef BENCHMARK_TESTING                           // 基准测试，测试算法时间
    auto start = std::chrono::system_clock::now(); // 记录开始时间
#endif

    if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_))
    {
      reset(); // 如果距上次调用时间超过重置周期，则重置控制器
    }
    last_time_called_ = clock_->now(); // 更新上次调用时间

    std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());        // 加锁以保护参数
    nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose); // 获取转换后的路径

    nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();                           // 获取代价地图指针
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex())); // 加锁代价地图

    geometry_msgs::msg::TwistStamped cmd = // 执行优化器以计算控制指令
        optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal_checker);

#ifdef BENCHMARK_TESTING
    auto end = std::chrono::system_clock::now();                                                // 记录结束时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); // 计算执行时间
    RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);                    // 输出控制循环执行时间
#endif

    // 遍历路径中的每个位姿
    // for (const auto &pose_stamped : transformed_plan.poses)
    // {
    //   // 获取位置信息
    //   const auto &pos = pose_stamped.pose.position;
    //   // 获取方向信息
    //   const auto &q = pose_stamped.pose.orientation;

    //   // 打印位置信息
    //   std::cout << "Position: x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;

    //   // 打印方向信息（四元数形式）
    //   std::cout << "Orientation: x=" << q.x << ", y=" << q.y
    //             << ", z=" << q.z << ", w=" << q.w << std::endl;

    //   auto quaternionToEulerAngles = [](const geometry_msgs::msg::Quaternion &q) -> std::tuple<double, double, double>
    //   {
    //     double roll, pitch, yaw;

    //     // 四元数到欧拉角的转换
    //     double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    //     double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    //     roll = std::atan2(sinr_cosp, cosr_cosp);

    //     double sinp = 2 * (q.w * q.y - q.z * q.x);
    //     if (std::abs(sinp) >= 1)
    //       pitch = std::copysign(M_PI / 2, sinp);
    //     else
    //       pitch = std::asin(sinp);

    //     double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    //     double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    //     yaw = std::atan2(siny_cosp, cosy_cosp);

    //     // 将弧度转换为度
    //     roll = roll * 180.0 / M_PI;
    //     pitch = pitch * 180.0 / M_PI;
    //     yaw = yaw * 180.0 / M_PI;

    //     return std::make_tuple(roll, pitch, yaw);
    //   };

    //   // 使用lambda函数
    //   double roll, pitch, yaw;
    //   std::tie(roll, pitch, yaw) = quaternionToEulerAngles(q);

    //   std::cout << "Roll: " << roll << " degrees\n";
    //   std::cout << "Pitch: " << pitch << " degrees\n";
    //   std::cout << "Yaw: " << yaw << " degrees\n";
    // }

    if (visualize_)
    {
      visualize(std::move(transformed_plan)); // 可视化转换后的路径
    }

    return cmd; // 返回控制指令
  }

  void MPPIController::visualize(nav_msgs::msg::Path transformed_plan)
  {
    trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories"); // 候选轨迹
    trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory");
    trajectory_visualizer_.visualize(std::move(transformed_plan));
  }

  void MPPIController::setPlan(const nav_msgs::msg::Path &path)
  {
    path_handler_.setPath(path);
  }

  void MPPIController::setSpeedLimit(const double &speed_limit, const bool &percentage)
  {
    optimizer_.setSpeedLimit(speed_limit, percentage);
  }

} // namespace nav2_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)

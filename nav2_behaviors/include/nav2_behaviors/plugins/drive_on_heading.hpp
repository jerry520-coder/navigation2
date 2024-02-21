// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2022 Joshua Wallace
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

#ifndef NAV2_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_HPP_
#define NAV2_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behaviors
{

/**
 * @class nav2_behaviors::DriveOnHeading
 * @brief An action server Behavior for spinning in
 */
template<typename ActionT = nav2_msgs::action::DriveOnHeading>
class DriveOnHeading : public TimedBehavior<ActionT>
{
public:
  /**
   * @brief A constructor for nav2_behaviors::DriveOnHeading
   */
  DriveOnHeading()
  : TimedBehavior<ActionT>(),
    feedback_(std::make_shared<typename ActionT::Feedback>()),
    command_x_(0.0),
    command_speed_(0.0),
    simulate_ahead_time_(0.0)
  {
  }

  ~DriveOnHeading() = default;

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */

  /**
 * @brief 运行行为的初始化
 * @param command 要执行的目标
 * @return 行为的状态
 */
  Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) override
  {
     // 检查是否在Y和Z轴上的目标不为零，因为只支持在X轴上移动
    if (command->target.y != 0.0 || command->target.z != 0.0) {
      RCLCPP_INFO(
        this->logger_,
        "DrivingOnHeading in Y and Z not supported, will only move in X.");
      return Status::FAILED;
    }

    // Ensure that both the speed and direction have the same sign
     // 确保速度和方向有相同的符号
    if (!((command->target.x > 0.0) == (command->speed > 0.0)) ) {
      RCLCPP_ERROR(this->logger_, "Speed and command sign did not match");
      return Status::FAILED;
    }

 // 保存命令中的目标X轴坐标、速度和时间允许
    command_x_ = command->target.x;
    command_speed_ = command->speed;
    command_time_allowance_ = command->time_allowance;

 // 设置结束时间，考虑时间允许的时间间隔
    end_time_ = this->steady_clock_.now() + command_time_allowance_;

 // 获取当前机器人姿态
    if (!nav2_util::getCurrentPose(
        initial_pose_, *this->tf_, this->global_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Initial robot pose is not available.");
      return Status::FAILED;
    }

    return Status::SUCCEEDED;
  }

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */

  /**
 * @brief 循环运行行为的函数
 * @return 行为的状态
 */
  Status onCycleUpdate()
  {
    // 计算距离目标时间的剩余时间
    rclcpp::Duration time_remaining = end_time_ - this->steady_clock_.now();
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
      // 如果超出时间允许范围且时间允许范围大于零，则停止机器人
      this->stopRobot();
      RCLCPP_WARN(
        this->logger_,
        "Exceeded time allowance before reaching the DriveOnHeading goal - Exiting DriveOnHeading");
      return Status::FAILED;
    }

// 获取当前机器人姿态
    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
        current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
      return Status::FAILED;
    }

 // 计算机器人当前位置与初始位置的差值
    double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
    double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
    double distance = hypot(diff_x, diff_y);

 // 更新反馈信息中的已行驶距离
    feedback_->distance_traveled = distance;
    this->action_server_->publish_feedback(feedback_);

 // 如果机器人已行驶距离大于或等于目标距离，则停止机器人并返回成功
    if (distance >= std::fabs(command_x_)) {
      this->stopRobot();
      return Status::SUCCEEDED;
    }

  // 创建指令速度消息并设置线速度
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.y = 0.0;
    cmd_vel->angular.z = 0.0;
    cmd_vel->linear.x = command_speed_;

// 获取机器人当前姿态
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = current_pose.pose.position.x;
    pose2d.y = current_pose.pose.position.y;
    pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

 // 检查模拟路径上是否有碰撞
    if (!isCollisionFree(distance, cmd_vel.get(), pose2d)) {
      // 如果有碰撞，则停止机器人并返回失败
      this->stopRobot();
      RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeading");
      return Status::FAILED;
    }

// 发布指令速度消息
    this->vel_pub_->publish(std::move(cmd_vel));

 // 返回运行中的状态
    return Status::RUNNING;
  }

protected:
  /**
   * @brief Check if pose is collision free
   * @param distance Distance to check forward
   * @param cmd_vel current commanded velocity
   * @param pose2d Current pose
   * @return is collision free or not
   */

  /**
 * @brief 检查姿态是否无碰撞（true 被用于表示无碰撞，而 false 表示有碰撞）
 * @param distance 距离前方的检查距离
 * @param cmd_vel 当前指令速度
 * @param pose2d 当前姿态
 * @return 是否无碰撞
 */
  bool isCollisionFree(
    const double & distance,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d)
  {
    // Simulate ahead by simulate_ahead_time_ in this->cycle_frequency_ increments
    // 以 this->cycle_frequency_ 的增量模拟 simulate_ahead_time_ 的时间
    int cycle_count = 0;
    double sim_position_change;// 模拟的位置变化
    const double diff_dist = abs(command_x_) - distance; // 计算与目标距离的差值
    const int max_cycle_count = static_cast<int>(this->cycle_frequency_ * simulate_ahead_time_); // 最大循环计数，用于限制模拟时间
    geometry_msgs::msg::Pose2D init_pose = pose2d; // 保存初始姿态
    bool fetch_data = true; // 控制是否从碰撞检测器中获取数据
   // 在最大周期计数内模拟运动
    while (cycle_count < max_cycle_count) {
       // 计算模拟的位置变化
      sim_position_change = cmd_vel->linear.x * (cycle_count / this->cycle_frequency_);
      pose2d.x = init_pose.x + sim_position_change * cos(init_pose.theta);
      pose2d.y = init_pose.y + sim_position_change * sin(init_pose.theta);
      cycle_count++;
      
    // 如果达到或超过目标距离，退出循环
      if (diff_dist - abs(sim_position_change) <= 0.) {
        break;
      }

    // 检查姿态是否无碰撞
      if (!this->collision_checker_->isCollisionFree(pose2d, fetch_data)) {
        return false;
      }
      fetch_data = false;
    }
    return true;
  }

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override
  {
    auto node = this->node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      "simulate_ahead_time", rclcpp::ParameterValue(2.0));
    node->get_parameter("simulate_ahead_time", simulate_ahead_time_);
  }

  typename ActionT::Feedback::SharedPtr feedback_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  double command_x_;
  double command_speed_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
  double simulate_ahead_time_;
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_HPP_

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

#ifndef NAV2_CORE__GOAL_CHECKER_HPP_
#define NAV2_CORE__GOAL_CHECKER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"


namespace nav2_core
{

/**
 * @class GoalChecker
 * @brief Function-object for checking whether a goal has been reached
 *
 * This class defines the plugin interface for determining whether you have reached
 * the goal state. This primarily consists of checking the relative positions of two poses
 * (which are presumed to be in the same frame). It can also check the velocity, as some
 * applications require that robot be stopped to be considered as having reached the goal.
 */

/**
 * @class GoalChecker
 * @brief 用于检查是否达到目标的函数对象
 *
 * 该类定义了一个插件接口，用于判断是否达到了目标状态。这主要包括检查两个姿态的相对位置
 * （假设它们在同一坐标系中）。它还可以检查速度，因为某些应用要求机器人停止才能被认为是达到了目标。
 */
class GoalChecker
{
public:
  typedef std::shared_ptr<nav2_core::GoalChecker> Ptr;

  virtual ~GoalChecker() {}

  /**
   * @brief Initialize any parameters from the NodeHandle
   * @param parent Node pointer for grabbing parameters
   */

  /**
 * @brief 从NodeHandle初始化任何参数
 * @param parent 用于获取参数的节点指针
 */
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void reset() = 0;

  /**
   * @brief Check whether the goal should be considered reached
   * @param query_pose The pose to check
   * @param goal_pose The pose to check against
   * @param velocity The robot's current velocity
   * @return True if goal is reached
   */

   /**
 * @brief 检查目标是否应该被认为已达到
 * @param query_pose 要检查的姿态
 * @param goal_pose 用来对比的目标姿态
 * @param velocity 机器人当前的速度
 * @return 如果目标已达到，则返回True
 */
  virtual bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) = 0;

  /**
   * @brief Get the maximum possible tolerances used for goal checking in the major types.
   * Any field without a valid entry is replaced with std::numeric_limits<double>::lowest()
   * to indicate that it is not measured. For tolerance across multiple entries
   * (e.x. XY tolerances), both fields will contain this value since it is the maximum tolerance
   * that each independent field could be assuming the other has no error (e.x. X and Y).
   * @param pose_tolerance The tolerance used for checking in Pose fields
   * @param vel_tolerance The tolerance used for checking velocity fields
   * @return True if the tolerances are valid to use
   */

  /**
 * @brief 获取用于目标检查的主要类型中可能的最大容忍度。
 * 任何没有有效条目的字段都将被替换为 std::numeric_limits<double>::lowest()
 * 来表示它不被测量。对于跨多个条目的容忍度
 * （例如，XY容忍度），两个字段都将包含此值，因为假设另一个没有误差时，
 * 每个独立字段可能具有的最大容忍度（例如，X和Y）。
 * @param pose_tolerance 用于检查Pose字段的容忍度
 * @param vel_tolerance 用于检查速度字段的容忍度
 * @return 如果容忍度有效可用，则返回True
 */
  virtual bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__GOAL_CHECKER_HPP_

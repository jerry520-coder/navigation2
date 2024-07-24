// Copyright (c) 2021 Samsung Research
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

#ifndef NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_TO_POSE_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_TO_POSE_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_bt_navigator/navigator.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace nav2_bt_navigator
{

/**
 * @class NavigateToPoseNavigator
 * @brief A navigator for navigating to a specified pose
 */
class NavigateToPoseNavigator
  : public nav2_bt_navigator::Navigator<nav2_msgs::action::NavigateToPose>
{
public:
  using ActionT = nav2_msgs::action::NavigateToPose;

  /**
   * @brief A constructor for NavigateToPoseNavigator
   */
  NavigateToPoseNavigator()
  : Navigator() {}

  /**
   * @brief A configure state transition to configure navigator's state
   * @param node Weakptr to the lifecycle node
   * @param odom_smoother Object to get current smoothed robot's speed
   */

  /**
 * @brief 配置状态转换以配置导航器的状态
 * @param node 指向生命周期节点的弱指针
 * @param odom_smoother 用于获取当前平滑机器人速度的对象
 */
  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  /**
   * @brief A cleanup state transition to remove memory allocated
   */
  bool cleanup() override;

  /**
   * @brief A subscription and callback to handle the topic-based goal published
   * from rviz
   * @param pose Pose received via atopic
   */

  /**
 * @brief 订阅并回调处理从 rviz 发布的topic-based的目标
 * @param pose 通过atopic接收的姿态
 */
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

  /**
   * @brief Get action name for this navigator
   * @return string Name of action server
   */
  std::string getName() override {return std::string("navigate_to_pose");}

  /**
   * @brief Get navigator's default BT
   * @param node WeakPtr to the lifecycle node
   * @return string Filepath to default XML
   */

  /**
 * @brief 获取导航器的默认 BT
 * @param node 指向生命周期节点的弱指针
 * @return string 默认 XML 文件的路径
 */
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   * @param goal Action template's goal message
   * @return bool if goal was received successfully to be processed
   */

  /**
 * @brief 当 BT action server接收到一个新目标时要调用的回调函数
 * 可用于检查目标是否有效，并将依赖于接收到的目标的值放在blackboard上
 * @param goal 动作模板的目标消息
 * @return bool 如果目标成功接收并准备处理
 */
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */

  /**
 * @brief 一个回调，定义了在 BT 迭代中发生的一次执行
 * 可用于发布action feedback
 */
  void onLoop() override;

  /**
   * @brief A callback that is called when a preempt is requested
   */

  /**
 * @brief 当请求抢占时调用的回调
 */
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that is called when a the action is completed, can fill in
   * action result message or indicate that this action is done.
   * @param result Action template result message to populate
   * @param final_bt_status Resulting status of the behavior tree execution that may be
   * referenced while populating the result.
   */

  /**
 * @brief 当动作完成时调用的回调，可以填充action result message或表明此动作已完成。
 * @param result 用于填充的Action template result message
 * @param final_bt_status 行为树执行的最终状态，可以在填充结果时参考
 */
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  /**
   * @brief Goal pose initialization on the blackboard
   * @param goal Action template's goal message to process
   */

  /**
 * @brief 在blackboard上初始化目标姿态
 * @param goal 要处理的Action template's goal message
 */
  void initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string goal_blackboard_id_;
  std::string path_blackboard_id_;

  // Odometry smoother object
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_TO_POSE_HPP_

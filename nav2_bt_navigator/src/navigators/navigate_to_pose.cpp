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

#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "nav2_bt_navigator/navigators/navigate_to_pose.hpp"

namespace nav2_bt_navigator
{

bool
NavigateToPoseNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  // 设置起始时间为0。
  start_time_ = rclcpp::Time(0);

  // 从弱指针中获取父节点的强引用。
  auto node = parent_node.lock();

  // 检查节点是否已经声明了"goal_blackboard_id"参数，如果没有，则声明并设置默认值为"goal"。
  if (!node->has_parameter("goal_blackboard_id")) {
    node->declare_parameter("goal_blackboard_id", std::string("goal"));
  }

  // 从节点获取"goal_blackboard_id"参数的值，并保存在成员变量goal_blackboard_id_中。
  goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();

  // 同样的过程，检查并声明"path_blackboard_id"参数，设置默认值为"path"，并获取其值保存在成员变量path_blackboard_id_中。
  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }
  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  // 保存传入的odom_smoother对象引用到成员变量odom_smoother_中，该对象用于获取当前速度。
  odom_smoother_ = odom_smoother;

  // 创建一个Action客户端，用于导航行动的执行。
  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  // 创建一个订阅者订阅"goal_pose"主题，当接收到目标位姿时调用onGoalPoseReceived回调函数。
  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPoseNavigator::onGoalPoseReceived, this, std::placeholders::_1));

  // 函数返回true，表示配置成功。
  return true;
}

std::string 
NavigateToPoseNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) // 接收一个弱指针作为参数，指向父节点
{
  std::string default_bt_xml_filename; // 定义一个字符串变量，用来存储默认的行为树文件路径
  auto node = parent_node.lock(); // 尝试从弱指针获取一个强引用，如果父节点已经不存在，这个操作会失败

  // 检查节点是否已经声明了"default_nav_to_pose_bt_xml"参数
  if (!node->has_parameter("default_nav_to_pose_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("nav2_bt_navigator"); // 获取包的共享目录路径
    // 声明并初始化"default_nav_to_pose_bt_xml"参数，设置默认的行为树XML文件路径
    node->declare_parameter<std::string>(
      "default_nav_to_pose_bt_xml",
      pkg_share_dir +
      "/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml");
  }

  // 获取"default_nav_to_pose_bt_xml"参数的值，并存储到default_bt_xml_filename变量中
  node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename; // 返回默认的行为树XML文件路径
}

bool
NavigateToPoseNavigator::cleanup()
{
  goal_sub_.reset();
  self_client_.reset();
  return true;
}

bool
NavigateToPoseNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  // 从目标中获取行为树文件名。
  auto bt_xml_filename = goal->behavior_tree;

  // 尝试加载指定的行为树文件。如果文件不存在，记录错误并返回false，取消导航。
  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  // 使用从目标中接收到的信息初始化目标位姿。
  initializeGoalPose(goal);

  // 如果上述步骤都成功执行，返回true。
  return true;
}

void
NavigateToPoseNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr /*result*/,
  const nav2_behavior_tree::BtStatus /*final_bt_status*/)
{
}

void
NavigateToPoseNavigator::onLoop()
{
    // action server feedback (pose, duration of task,
    // number of recoveries, and distance remaining to goal)
    // 创建一个动作反馈消息的共享指针。
    auto feedback_msg = std::make_shared<ActionT::Feedback>();

    // 定义一个PoseStamped类型的变量current_pose，用于存储当前位姿。
    geometry_msgs::msg::PoseStamped current_pose;
    // 调用nav2_util::getCurrentPose函数获取当前位姿。
    nav2_util::getCurrentPose(
      current_pose, *feedback_utils_.tf,
      feedback_utils_.global_frame, feedback_utils_.robot_frame,
      feedback_utils_.transform_tolerance);

    // 获取行为树的黑板对象。
    auto blackboard = bt_action_server_->getBlackboard();

    try {
      // 从黑板获取当前路径。
      nav_msgs::msg::Path current_path;
      blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, current_path);

      // 找到当前路径上与当前位姿最近的点。
      auto find_closest_pose_idx =
        [&current_pose, &current_path]() {
          size_t closest_pose_idx = 0;
          double curr_min_dist = std::numeric_limits<double>::max();
          for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
            double curr_dist = nav2_util::geometry_utils::euclidean_distance(
              current_pose, current_path.poses[curr_idx]);
            if (curr_dist < curr_min_dist) {
              curr_min_dist = curr_dist;
              closest_pose_idx = curr_idx;
            }
          }
          return closest_pose_idx;
        };

      // 计算从最近的点到路径终点的剩余距离。
      double distance_remaining =
        nav2_util::geometry_utils::calculate_path_length(current_path, find_closest_pose_idx());

      // 初始化估计剩余时间。
      rclcpp::Duration estimated_time_remaining = rclcpp::Duration::from_seconds(0.0);

      // 获取当前速度。
      geometry_msgs::msg::Twist current_odom = odom_smoother_->getTwist();
      double current_linear_speed = std::hypot(current_odom.linear.x, current_odom.linear.y);

      // 如果当前线速度大于1cm/s且剩余距离超过10cm，计算估计的剩余时间。
      if ((std::abs(current_linear_speed) > 0.01) && (distance_remaining > 0.1)) {
        estimated_time_remaining =
          rclcpp::Duration::from_seconds(distance_remaining / std::abs(current_linear_speed));
      }

      // 设置反馈消息的剩余距离和估计剩余时间。
      feedback_msg->distance_remaining = distance_remaining;
      feedback_msg->estimated_time_remaining = estimated_time_remaining;
    } catch (...) {
      // 忽略异常。
    }

    // 从黑板获取恢复次数并设置到反馈消息。
    int recovery_count = 0;
    blackboard->get<int>("number_recoveries", recovery_count);
    feedback_msg->number_of_recoveries = recovery_count;

    // 设置反馈消息的当前位姿和导航时间。
    feedback_msg->current_pose = current_pose;
    feedback_msg->navigation_time = clock_->now() - start_time_;

    // 发布反馈消息。
    bt_action_server_->publishFeedback(feedback_msg);
}

void
NavigateToPoseNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  // 当接收到目标抢占请求时，首先打印日志信息。
  RCLCPP_INFO(logger_, "Received goal preemption request");

// 检查抢占请求的行为树文件是否与当前目标执行的行为树文件相同，或者抢占请求的行为树字段为空并且当前目标执行的是默认行为树文件。
  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
  // 如果符合上述条件之一，表示可以接受抢占请求：
  // - 如果待处理的目标请求使用的是与当前目标相同的行为树文件，或者
  // - 如果待处理目标的行为树字段为空，且当前目标正在执行默认的行为树文件。
  // 在这种情况下，初始化新的目标位姿，并接受待处理的目标。
    initializeGoalPose(bt_action_server_->acceptPendingGoal());
  } else {
  // 如果抢占请求指定了一个与当前执行的行为树文件不同的行为树文件，打印警告信息。
  // 说明抢占请求被拒绝，因为请求了一个新的行为树文件，这种情况下应该取消当前目标而不是真正的抢占。
  // 如果需要使用不同的行为树文件，应该先取消当前目标，然后发送一个新的动作请求。
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
        // 终止待处理的目标。
    bt_action_server_->terminatePendingGoal();
  }
}

void
NavigateToPoseNavigator::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal)
{
  // 定义一个PoseStamped类型的变量current_pose，用于存储当前位姿。
  geometry_msgs::msg::PoseStamped current_pose;
  
  // 调用nav2_util::getCurrentPose函数获取当前位姿，这个函数需要tf变换树来确定当前位置。
  nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);

  // 使用RCLCPP_INFO宏记录一条日志信息，包含当前位置和目标位置的坐标。
  RCLCPP_INFO(
    logger_, "Begin navigating from current location (%.2f, %.2f) to (%.2f, %.2f)",
    current_pose.pose.position.x, current_pose.pose.position.y,
    goal->pose.pose.position.x, goal->pose.pose.position.y);

  // 重置开始时间为当前时间，表示一个新的导航动作开始。
  start_time_ = clock_->now();
  
  // 获取行为树的黑板对象。
  auto blackboard = bt_action_server_->getBlackboard();
  
  // 在黑板上设置“number_recoveries”的值为0，这表示重置恢复行动的次数。
  blackboard->set<int>("number_recoveries", 0);  // NOLINT

  // 在黑板上更新目标位姿，使用从外部接收到的目标位姿。
  blackboard->set<geometry_msgs::msg::PoseStamped>(goal_blackboard_id_, goal->pose);
}

void
NavigateToPoseNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  ActionT::Goal goal;
  goal.pose = *pose;
  self_client_->async_send_goal(goal);
}

}  // namespace nav2_bt_navigator

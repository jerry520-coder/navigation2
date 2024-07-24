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

#ifndef NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_
#define NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_

#include <vector>
#include <memory>
#include <utility>

#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/msg/behavior_tree_status_change.h"
#include "tf2_ros/buffer_interface.h"

namespace nav2_behavior_tree
{

/**
 * @brief A class to publish BT logs on BT status change
 */
class RosTopicLogger : public BT::StatusChangeLogger
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RosTopicLogger
   * @param ros_node Weak pointer to parent rclcpp::Node
   * @param tree BT to monitor
   */
  RosTopicLogger(const rclcpp::Node::WeakPtr & ros_node, const BT::Tree & tree)
  : StatusChangeLogger(tree.rootNode())
  {
    auto node = ros_node.lock();
    clock_ = node->get_clock();
    logger_ = node->get_logger();
    log_pub_ = node->create_publisher<nav2_msgs::msg::BehaviorTreeLog>(
      "behavior_tree_log",
      rclcpp::QoS(10));
  }

  /**
   * @brief Callback function which is called each time BT changes status
   * @param timestamp Timestamp of BT status change
   * @param node Node that changed status
   * @param prev_status Previous status of the node
   * @param status Current status of the node
   * @note 专门用于处理特定的事件，即行为树节点的状态变化。
   */
  void callback(
    BT::Duration timestamp,
    const BT::TreeNode & node,
    BT::NodeStatus prev_status,
    BT::NodeStatus status) override
  {
      nav2_msgs::msg::BehaviorTreeStatusChange event; // 创建一个消息对象，用于记录状态变化。
      
    // BT timestamps are a duration since the epoch. Need to convert to a time_point
    // before converting to a msg.
      // BT时间戳是自纪元以来的持续时间，需要转换为时间点（time_point），然后再转换为消息格式。
      event.timestamp = tf2_ros::toMsg(tf2::TimePoint(timestamp));
      // 记录节点名称。
      event.node_name = node.name();
      // 记录节点之前的状态，将枚举值转换为字符串形式。
      event.previous_status = toStr(prev_status, false);
      // 记录节点当前的状态，同样将枚举值转换为字符串形式。
      event.current_status = toStr(status, false);
      // 将这个状态变化事件添加到事件日志中。
      event_log_.push_back(std::move(event));

      // 使用RCLCPP_DEBUG宏记录一条调试信息到ROS 2节点的日志中。
      // 这条信息包含时间戳（转换为秒，并保留三位小数）、节点名称、之前的状态以及当前的状态。
      // 注意这里使用了toStr函数的另一个版本，传入true以获取更详细的状态字符串。
      RCLCPP_DEBUG(
        logger_, "[%.3f]: %25s %s -> %s",
        std::chrono::duration<double>(timestamp).count(), // 将时间戳转换为秒。
        node.name().c_str(),                              // 节点名称。
        toStr(prev_status, true).c_str(),                 // 之前的状态（详细字符串形式）。
        toStr(status, true).c_str() );                    // 当前的状态（详细字符串形式）。
  }

  /**
   * @brief Clear log buffer if any
   */
  void flush() override
  {
    if (!event_log_.empty()) {
      auto log_msg = std::make_unique<nav2_msgs::msg::BehaviorTreeLog>();
      log_msg->timestamp = clock_->now();
      log_msg->event_log = event_log_;
      log_pub_->publish(std::move(log_msg));
      event_log_.clear();
    }
  }

protected:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("bt_navigator")};
  rclcpp::Publisher<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr log_pub_;
  std::vector<nav2_msgs::msg::BehaviorTreeStatusChange> event_log_;
};

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_

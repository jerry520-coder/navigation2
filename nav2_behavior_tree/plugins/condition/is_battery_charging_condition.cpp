// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#include <string>

#include "nav2_behavior_tree/plugins/condition/is_battery_charging_condition.hpp"

namespace nav2_behavior_tree
{

// 用于判断电池是否正在充电。它从黑板获取ROS节点共享指针，为该节点创建一个回调组，并将电池状态话题的订阅者添加到该回调组。
// 订阅话题时，如果电池状态发生变化，则调用batteryCallback函数来更新电池充电状态。
IsBatteryChargingCondition::IsBatteryChargingCondition(
  const std::string & condition_name,  // 条件节点的名称
  const BT::NodeConfiguration & conf)  // 行为树节点的配置
: BT::ConditionNode(condition_name, conf), // 初始化BT::ConditionNode的基类部分
  battery_topic_("/battery_status"), // 默认订阅的话题名称
  is_battery_charging_(false) // 默认电池未在充电
{
  // 尝试从输入中获取“battery_topic”，如果提供则更新battery_topic_
  getInput("battery_topic", battery_topic_);
  
  // 从黑板中获取rclcpp::Node的共享指针
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 为节点创建一个互斥的回调组，不自动添加到执行器中
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);

  // 将创建的回调组添加到执行器中
  callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  // 设置订阅选项，指定回调组
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  // 创建订阅，订阅电池状态消息
  // 当收到消息时，调用batteryCallback函数
  battery_sub_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_, // 订阅的话题
    rclcpp::SystemDefaultsQoS(), // 使用系统默认的服务质量设置
    std::bind(&IsBatteryChargingCondition::batteryCallback, this, std::placeholders::_1), // 绑定的回调函数
    sub_option); // 订阅选项，包括回调组
}

BT::NodeStatus IsBatteryChargingCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_battery_charging_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsBatteryChargingCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  is_battery_charging_ =
    (msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsBatteryChargingCondition>("IsBatteryCharging");
}

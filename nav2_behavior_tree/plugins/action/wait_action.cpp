// Copyright (c) 2018 Samsung Research America
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
#include <memory>

#include "nav2_behavior_tree/plugins/action/wait_action.hpp"

namespace nav2_behavior_tree
{

WaitAction::WaitAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Wait>(xml_tag_name, action_name, conf)
{
  // 定义一个整型变量duration，用于存储等待的持续时间。
  int duration;
  // 使用getInput方法从节点配置中获取"wait_duration"参数的值，并存入duration变量。
  getInput("wait_duration", duration);
  // 如果duration的值小于或等于0，则打印警告信息，并将duration的值转为正数。
  if (duration <= 0) {
    RCLCPP_WARN(
      node_->get_logger(), "Wait duration is negative or zero "
      "(%i). Setting to positive.", duration);
    duration *= -1;
  }

  // 将goal_的time成员的sec字段设置为duration，goal_是BtActionNode基类中定义的成员变量，用于存储动作的目标。
  goal_.time.sec = duration;
}

void WaitAction::on_tick()
{
  // 每次这个节点被tick时，调用increment_recovery_count函数。
  // 这个函数的具体作用依赖于它的实现，但通常用于增加某种形式的恢复计数器。
  // 在某些场景中，这可能用于跟踪节点被tick的次数，或者是用于实现某种恢复策略。
  increment_recovery_count();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  // NodeBuilder是一个函数对象，它接收节点的名称和配置作为参数，返回一个新创建的行为树节点的智能指针。
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::WaitAction>(name, "wait", config);
    };
// 将上面定义的builder注册到工厂中，以创建Wait类型的节点。当行为树解析器在行为树定义中遇到Wait节点时，它会使用这个builder来创建WaitAction节点的实例
  factory.registerBuilder<nav2_behavior_tree::WaitAction>("Wait", builder);
}
// BTCPP_EXPORT是一个由BehaviorTree.CPP框架定义的宏，用于确保函数能够在动态链接库(DLL)中被导出，使得函数能在DLL外部被调用。

// Copyright (c) 2018 Intel Corporation
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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"

namespace nav2_behavior_tree
{

FollowPathAction::FollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
{
}

void FollowPathAction::on_tick()
{
  getInput("path", goal_.path);
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);
}

void FollowPathAction::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback>/*feedback*/)
{
  // 抓取新路径
  nav_msgs::msg::Path new_path;
  getInput("path", new_path); // 从行为树的输入端口获取新路径

  // 检查新路径是否与当前目标路径不同
  if (goal_.path != new_path) {
    // 如果新路径与当前路径不同，则更新目标路径，并标记目标已更新
    goal_.path = new_path;
    goal_updated_ = true;
  }

  // 抓取新控制器ID
  std::string new_controller_id;
  getInput("controller_id", new_controller_id); // 从行为树的输入端口获取新控制器ID

  // 检查新控制器ID是否与当前目标控制器ID不同
  if (goal_.controller_id != new_controller_id) {
    // 如果新控制器ID与当前控制器ID不同，则更新目标控制器ID，并标记目标已更新
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  // 抓取新目标检查器ID
  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id); // 从行为树的输入端口获取新目标检查器ID

  // 检查新目标检查器ID是否与当前目标目标检查器ID不同
  if (goal_.goal_checker_id != new_goal_checker_id) {
    // 如果新目标检查器ID与当前目标检查器ID不同，则更新目标目标检查器ID，并标记目标已更新
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::FollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FollowPathAction>(
    "FollowPath", builder);
}

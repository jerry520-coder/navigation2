// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
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

#include "nav2_behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace nav2_behavior_tree
{

// 加载行为树节点的插件。插件机制允许行为树节点的功能动态地扩展，而不需要修改现有的代码库。
BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries)
{
  // BehaviorTreeEngine类能够在初始化时加载指定的插件库，使得行为树能够使用这些库中定义的节点。

  // 创建一个BT::SharedLibrary对象，用于加载共享库。
  BT::SharedLibrary loader;
  
  // 遍历所有提供的插件库名称。
  for (const auto & p : plugin_libraries) {
    // 使用BT::BehaviorTreeFactory类的registerFromPlugin方法加载每一个插件。
    // loader.getOSName(p)方法用于获取插件库文件的操作系统特定的名称，例如在Linux上，可能会将"my_plugin"转换为"libmy_plugin.so"。
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

BtStatus
BehaviorTreeEngine::run(
  BT::Tree * tree,  // 指向行为树对象的指针。
  std::function<void()> onLoop,  // 在每次循环中调用的函数。
  std::function<bool()> cancelRequested,  // 检查是否有取消请求的函数。
  std::chrono::milliseconds loopTimeout)  // 循环超时时间。
{
  // 创建一个循环频率控制器，用于控制循环执行的频率。
  rclcpp::WallRate loopRate(loopTimeout);
  // 初始化行为树节点的执行结果为RUNNING。
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // 捕获可能发生的异常。
  try {
    // 在ROS环境正常运行且行为树节点状态为RUNNING时，进入循环。
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      // 如果检查到有取消请求，则停止行为树的根节点并返回取消状态。
      if (cancelRequested()) {
        tree->rootNode()->halt();
        return BtStatus::CANCELED;
      }

      // 执行行为树的根节点
      result = tree->tickRoot();

      // 调用循环中的函数。
      onLoop();

      // 如果循环执行的实际时间超过了预设的频率，则记录一条警告信息。
      if (!loopRate.sleep()) {
        RCLCPP_WARN(
          rclcpp::get_logger("BehaviorTreeEngine"),
          "Behavior Tree tick rate %0.2f was exceeded!",
          1.0 / (loopRate.period().count() * 1.0e-9));
      }
    }
  } catch (const std::exception & ex) {
    // 如果在执行过程中捕获到异常，则记录错误信息并返回失败状态。
    RCLCPP_ERROR(
      rclcpp::get_logger("BehaviorTreeEngine"),
      "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
    return BtStatus::FAILED;
  }

  // 根据行为树执行的最终结果返回相应的状态。
  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree
BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree
BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromFile(file_path, blackboard);
}


void
BehaviorTreeEngine::addGrootMonitoring(
  BT::Tree * tree,
  uint16_t publisher_port,
  uint16_t server_port,
  uint16_t max_msg_per_second)
{
  // This logger publish status changes using ZeroMQ. Used by Groot
  groot_monitor_ = std::make_unique<BT::PublisherZMQ>(
    *tree, max_msg_per_second, publisher_port,
    server_port);
}

void
BehaviorTreeEngine::resetGrootMonitor()
{
  if (groot_monitor_) {
    groot_monitor_.reset();
  }
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
// 目的是停止行为树中所有节点的执行，将整个树重置回初始状态。当需要中断行为树的执行或者准备重新运行一个行为树时，这个函数会非常有用。
void
BehaviorTreeEngine::haltAllActions(BT::TreeNode * root_node)
{
  // 如果root_node为空，即没有传入有效的根节点，直接返回，不执行任何操作。
  if (!root_node) {
    return;
  }

  // 首先直接调用根节点的halt方法，这个调用会向整个行为树传播停止信号。
  root_node->halt();

  // 定义一个lambda函数visitor，它接受一个指向TreeNode的指针作为参数。
  // 这个函数检查每个节点的状态，如果节点的状态是RUNNING，则调用其halt方法。
  auto visitor = [](BT::TreeNode * node) {
      if (node->status() == BT::NodeStatus::RUNNING) {
        node->halt();
      }
    };

  // 使用BT::applyRecursiveVisitor函数遍历整个行为树。
  // 对树中的每个节点应用visitor函数，以确保所有正在运行的节点都被停止。
  // 这是一种保险措施，以确保树中的所有节点都被正确地停止。
  BT::applyRecursiveVisitor(root_node, visitor);
}

}  // namespace nav2_behavior_tree

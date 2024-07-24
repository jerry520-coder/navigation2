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

#include <string>
#include "nav2_behavior_tree/plugins/control/recovery_node.hpp"

namespace nav2_behavior_tree
{

RecoveryNode::RecoveryNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0),
  number_of_retries_(1),
  retry_count_(0)
{
  getInput("number_of_retries", number_of_retries_);
}

BT::NodeStatus RecoveryNode::tick()
{
  // 获取子节点的数量。
  const unsigned children_count = children_nodes_.size();

  // 如果子节点数量不等于2，则抛出异常。RecoveryNode设计为必须有两个子节点。
  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }

  // 设置当前节点的状态为RUNNING，表示正在运行。
  setStatus(BT::NodeStatus::RUNNING);

  // 当当前子节点索引小于子节点数量，并且重试次数不超过设定的最大重试次数时，循环执行。
  while (current_child_idx_ < children_count && retry_count_ <= number_of_retries_) {
    // 获取当前子节点。
    TreeNode * child_node = children_nodes_[current_child_idx_];
    // 执行当前子节点的tick函数，并获取其状态。
    const BT::NodeStatus child_status = child_node->executeTick();

    // 如果当前是第一个子节点。
    if (current_child_idx_ == 0) {
      switch (child_status) {
        // 如果子节点执行成功。
        case BT::NodeStatus::SUCCESS:
          {
            // 重置节点状态并返回成功。
            halt();
            return BT::NodeStatus::SUCCESS;
          }

        // 如果子节点执行失败。
        case BT::NodeStatus::FAILURE:
          {
            // 如果重试次数小于最大重试次数。
            if (retry_count_ < number_of_retries_) {
              // 停止第一个子节点，并在下一次迭代中执行第二个子节点。
              ControlNode::haltChild(0);
              current_child_idx_++;
              break;
            } else {
              // 如果达到最大重试次数，重置节点状态并返回失败。
              halt();
              return BT::NodeStatus::FAILURE;
            }
          }

        // 如果子节点状态为运行中。
        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        // 其他情况，抛出逻辑错误。
        default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch
    } else if (current_child_idx_ == 1) { // 如果当前是第二个子节点。
      switch (child_status) {
        // 如果子节点执行成功。
        case BT::NodeStatus::SUCCESS:
          {
            // 停止第二个子节点，增加重试计数，下一次迭代中执行第一个子节点。
            ControlNode::haltChild(1);
            retry_count_++;
            current_child_idx_--;
          }
          break;

        // 如果子节点执行失败。
        case BT::NodeStatus::FAILURE:
          {
            // 重置节点状态并返回失败。
            halt();
            return BT::NodeStatus::FAILURE;
          }

        // 如果子节点状态为运行中。
        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        // 其他情况，抛出逻辑错误。
        default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch
    }
  }  // end while loop

  // 如果循环结束，表示重试次数超过最大限制，重置节点状态并返回失败。
  halt();
  return BT::NodeStatus::FAILURE;
}

void RecoveryNode::halt()
{
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
}

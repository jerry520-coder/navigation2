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

#include <memory>

#include "nav2_util/node_thread.hpp"

namespace nav2_util
{

NodeThread::NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
: node_(node_base)
{
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
   // 创建一个独特的线程来处理节点的回调
  thread_ = std::make_unique<std::thread>(
    [&]()
    {
      executor_->add_node(node_);// 向执行器添加节点
      executor_->spin();// 在当前线程中启动执行器的事件循环，处理所有回调
      executor_->remove_node(node_); // 事件循环结束后，从执行器中移除节点
    });
}

NodeThread::NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor)
: executor_(executor)
{
  thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
}

NodeThread::~NodeThread()
{
  executor_->cancel();
  thread_->join();
}

}  // namespace nav2_util

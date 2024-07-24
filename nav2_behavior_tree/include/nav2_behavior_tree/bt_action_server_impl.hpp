// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_

#include <memory>
#include <string>
#include <fstream>
#include <set>
#include <exception>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behavior_tree
{

template<class ActionT>
BtActionServer<ActionT>::BtActionServer(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & action_name,
  const std::vector<std::string> & plugin_lib_names,
  const std::string & default_bt_xml_filename,
  OnGoalReceivedCallback on_goal_received_callback,
  OnLoopCallback on_loop_callback,
  OnPreemptCallback on_preempt_callback,
  OnCompletionCallback on_completion_callback)
: action_name_(action_name),
  default_bt_xml_filename_(default_bt_xml_filename),
  plugin_lib_names_(plugin_lib_names),
  node_(parent),
  on_goal_received_callback_(on_goal_received_callback),
  on_loop_callback_(on_loop_callback),
  on_preempt_callback_(on_preempt_callback),
  on_completion_callback_(on_completion_callback)
{
  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare this node's parameters
  if (!node->has_parameter("bt_loop_duration")) {
    node->declare_parameter("bt_loop_duration", 10);
  }
  if (!node->has_parameter("default_server_timeout")) {
    node->declare_parameter("default_server_timeout", 20);
  }

  if (!node->has_parameter("enable_groot_monitoring")) {
  node->declare_parameter("enable_groot_monitoring", true);
  }
  if (!node->has_parameter("groot_zmq_publisher_port")) {
    node->declare_parameter("groot_zmq_publisher_port", 1666);
  }
  if (!node->has_parameter("groot_zmq_server_port")) {
    node->declare_parameter("groot_zmq_server_port", 1667);
  }
}

template<class ActionT>
BtActionServer<ActionT>::~BtActionServer()
{}

template<class ActionT>
bool BtActionServer<ActionT>::on_configure()
{
    // 尝试获取一个弱引用的node_对象的共享指针
    auto node = node_.lock();
    // 如果获取失败，则抛出运行时错误
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    // 将客户端节点命名为动作名称，将其中的'/'替换为'_'以适应ROS命名规则
    std::string client_node_name = action_name_;
    std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
    // 添加后缀 '_rclcpp_node' 以保持参数文件的一致性
    auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args",
        "-r",
        std::string("__node:=") +
        std::string(node->get_name()) + "_" + client_node_name + "_rclcpp_node",
        "--"});

    // 支持从rviz处理基于话题的目标位置
    client_node_ = std::make_shared<rclcpp::Node>("_", options);

    // 声明通用客户端节点参数，以便与行为树节点共享
    // 如果参数尚未声明，并且可能被外部应用程序使用，则声明参数
    // 然后将主节点的所有参数复制到客户端节点，以供行为树节点获取
    nav2_util::declare_parameter_if_not_declared(
      node, "global_frame", rclcpp::ParameterValue(std::string("map")));
    nav2_util::declare_parameter_if_not_declared(
      node, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
    nav2_util::declare_parameter_if_not_declared(
      node, "transform_tolerance", rclcpp::ParameterValue(0.1));
    nav2_util::copy_all_parameters(node, client_node_);

    // 创建动作服务器，绑定执行回调函数executeCallback
    action_server_ = std::make_shared<ActionServer>(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_waitables_interface(),
      action_name_, std::bind(&BtActionServer<ActionT>::executeCallback, this));

  // Get parameter for monitoring with Groot via ZMQ Publisher
  node->get_parameter("enable_groot_monitoring", enable_groot_monitoring_);
  node->get_parameter("groot_zmq_publisher_port", groot_zmq_publisher_port_);
  node->get_parameter("groot_zmq_server_port", groot_zmq_server_port_);

    // 获取行为树超时参数
    int timeout;
    node->get_parameter("bt_loop_duration", timeout);
    bt_loop_duration_ = std::chrono::milliseconds(timeout);
    node->get_parameter("default_server_timeout", timeout);
    default_server_timeout_ = std::chrono::milliseconds(timeout);

    // 创建注册自定义节点并执行行为树的类
    bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

    // 创建将被树中所有节点共享的黑板
    blackboard_ = BT::Blackboard::create();

    // 在黑板上设置项
    blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // 将客户端节点共享给树中的节点
    blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  // 设置服务器超时
    blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);  // 设置行为树循环持续时间

    // 配置成功完成，返回true
    return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_activate()
{
  if (!loadBehaviorTree(default_bt_xml_filename_)) {
    RCLCPP_ERROR(logger_, "Error loading XML file: %s", default_bt_xml_filename_.c_str());
    return false;
  }
  action_server_->activate();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_deactivate()
{
  action_server_->deactivate();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_cleanup()
{
  client_node_.reset();
  action_server_.reset();
  topic_logger_.reset();
  plugin_lib_names_.clear();
  current_bt_xml_filename_.clear();
  blackboard_.reset();
  bt_->haltAllActions(tree_.rootNode());
  bt_->resetGrootMonitor();
  bt_.reset();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::loadBehaviorTree(const std::string & bt_xml_filename)
{
  // Empty filename is default for backward compatibility
  // 空文件名用于向后兼容，默认使用默认的行为树XML文件名
  auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

  // Use previous BT if it is the existing one
  // 如果当前行为树XML文件名与给定文件名相同，则使用先前的行为树，无需重新加载
  if (current_bt_xml_filename_ == filename) {
    RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml is already loaded");
    return true;
  }

  // if a new tree is created, than the ZMQ Publisher must be destroyed
  bt_->resetGrootMonitor();

  // Read the input BT XML from the specified file into a string
  // 从指定文件中读取输入行为树XML并存储为字符串
  std::ifstream xml_file(filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(logger_, "Couldn't open input XML file: %s", filename.c_str());
    return false;
  }

// 从文件中创建一个字符串以存储XML内容
  auto xml_string = std::string(
    std::istreambuf_iterator<char>(xml_file), // 使用文件迭代器读取字符
    std::istreambuf_iterator<char>());        // 结束文件迭代

  // Create the Behavior Tree from the XML input
  // 从XML输入创建行为树
  try {
    tree_ = bt_->createTreeFromText(xml_string, blackboard_);

    // 为行为树中的每个blackboard设置节点、服务器超时和行为树循环周期
    for (auto & blackboard : tree_.blackboard_stack) {
      blackboard->set<rclcpp::Node::SharedPtr>("node", client_node_);
      blackboard->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception when loading BT: %s", e.what());
    return false;
  }

  // Enable monitoring with Groot
  if (enable_groot_monitoring_) {
    // optionally add max_msg_per_second = 25 (default) here
    try {
      bt_->addGrootMonitoring(&tree_, groot_zmq_publisher_port_, groot_zmq_server_port_);
    } catch (const std::logic_error & e) {
      RCLCPP_ERROR(logger_, "ZMQ already enabled, Error: %s", e.what());
    }
  }

 // 创建用于记录行为树的RosTopicLogger实例
  topic_logger_ = std::make_unique<RosTopicLogger>(client_node_, tree_);

  // 更新当前行为树XML文件名
  current_bt_xml_filename_ = filename;
  return true;
}

template<class ActionT>
void BtActionServer<ActionT>::executeCallback()
{
    // 如果接收到的目标不符合预期，则终止当前目标并返回
    if (!on_goal_received_callback_(action_server_->get_current_goal())) {
      action_server_->terminate_current();
      return;
    }

    // 定义一个检查取消请求的lambda表达式
    auto is_canceling = [&]() {
        // 如果动作服务器为null，说明已经不可用，需要取消
        if (action_server_ == nullptr) {
          RCLCPP_DEBUG(logger_, "Action server unavailable. Canceling.");
          return true;
        }
        // 如果动作服务器不处于活动状态，也需要取消
        if (!action_server_->is_server_active()) {
          RCLCPP_DEBUG(logger_, "Action server is inactive. Canceling.");
          return true;
        }
        // 返回是否有取消请求
        return action_server_->is_cancel_requested();
      };

    // 定义一个执行循环中的操作的lambda表达式
    auto on_loop = [&]() {
        // 如果有抢占请求，并且定义了抢占回调，则调用抢占回调
        if (action_server_->is_preempt_requested() && on_preempt_callback_) {
          on_preempt_callback_(action_server_->get_pending_goal());
        }
        // 刷新主题日志
        topic_logger_->flush();
        // 调用循环回调
        on_loop_callback_();
      };

    // 执行之前在配置步骤中创建的行为树
    nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);

    // 确保行为树不处于之前执行的运行状态
    // 注意：如果所有的控制节点都正确实现，这一步是不需要的
    bt_->haltAllActions(tree_.rootNode());

    // 创建一个结果消息的共享指针，供服务器填充结果信息或简单地标记动作完成
    auto result = std::make_shared<typename ActionT::Result>();
    // 调用完成回调，允许用户根据行为树的执行结果修改结果消息
    on_completion_callback_(result, rc);

    // 根据行为树的执行结果，处理相应的动作服务器状态变更
    switch (rc) {
      case nav2_behavior_tree::BtStatus::SUCCEEDED:
        RCLCPP_INFO(logger_, "Goal succeeded");
        // 如果成功，设置动作服务器的当前目标为成功状态
        action_server_->succeeded_current(result);
        break;

      case nav2_behavior_tree::BtStatus::FAILED:
        RCLCPP_ERROR(logger_, "Goal failed");
        // 如果失败，终止当前目标
        action_server_->terminate_current(result);
        break;

      case nav2_behavior_tree::BtStatus::CANCELED:
        RCLCPP_INFO(logger_, "Goal canceled");
        // 如果取消，终止所有目标
        action_server_->terminate_all(result);
        break;
    }
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_

// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_behavior_tree
{

using namespace std::chrono_literals;  // NOLINT 可以直接使用如10ms、1s、2h等字面量来创建对应的std::chrono持续时间对象，而不需要使用较为繁琐的构造函数或工厂函数。

/**
 * @brief Abstract class representing a service based BT node
 * @tparam ServiceT Type of service
 */

/**
 * @brief 代表基于服务的行为树节点的抽象类
 * @tparam ServiceT 服务的类型
 */
template<class ServiceT>
class BtServiceNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::BtServiceNode constructor
   * @param service_node_name BT node name
   * @param conf BT node configuration
   * @param service_name Optional service name this node creates a client for instead of from input port
   */
/**
 * @brief 一个nav2_behavior_tree::BtServiceNode构造函数
 * @param service_node_name 行为树节点名称
 * @param conf 行为树节点配置
 * @param service_name 可选，此节点创建客户端以调用的服务名称，而不是从输入端口获取
 */
BtServiceNode(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf,
  const std::string & service_name = "")
: BT::ActionNodeBase(service_node_name, conf), service_name_(service_name), service_node_name_(
    service_node_name)
{
  // 从黑板中获取ROS节点的共享指针
  node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
  // 创建回调组
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  // 将回调组添加到执行器
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 从黑板中获取所需项
  bt_loop_duration_ =
    config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
  server_timeout_ =
    config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  // 获取输入端口的“server_timeout”，如果有的话
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

  // 现在我们有了ROS节点，可以创建这个行为树服务的服务客户端了
  // 获取输入端口的“service_name”，如果有的话
  getInput("service_name", service_name_);
  service_client_ = node_->create_client<ServiceT>(
    service_name_,
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);

  // 创建一个空的服务请求
  request_ = std::make_shared<typename ServiceT::Request>();

  // 确保服务端确实存在
  RCLCPP_DEBUG(
    node_->get_logger(), "Waiting for \"%s\" service",
    service_name_.c_str());
  // 等待服务可用，超时时间为1秒
  if (!service_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(
      node_->get_logger(), "\"%s\" service server not available after waiting for 1 s",
      service_node_name.c_str());
    // 如果服务不可用，抛出运行时错误
    throw std::runtime_error(
            std::string(
              "Service server %s not available",
              service_node_name.c_str()));
  }

  RCLCPP_DEBUG(
    node_->get_logger(), "\"%s\" BtServiceNode initialized",
    service_node_name_.c_str());
}

  BtServiceNode() = delete;

  virtual ~BtServiceNode()
  {
  }

  /**
   * @brief Any subclass of BtServiceNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */

  /**
 * @brief 任何接受参数的BtServiceNode的子类都必须提供一个providedPorts方法，并在其中调用providedBasicPorts。
 * @param addition 需要添加到行为树端口列表中的附加端口
 * @return BT::PortsList 包含基本端口以及节点特定的端口
 */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   * @note  这里，它被重写（override），以实现特定的功能。
   */
BT::NodeStatus tick() override
{
  // 检查是否已经发送了请求。如果没有发送请求，则执行请求发送的逻辑。
  if (!request_sent_) {
    // 首先，重置should_send_request_标志，允许在on_tick用户定义的回调中设置它。
    // 这个标志用来决定是否应该发送请求。
    should_send_request_ = true;

    // 调用用户定义的on_tick回调函数。这个回调函数可以修改should_send_request_的值，
    // 从而影响是否发送请求的决定。
    on_tick();

    // 检查回调函数执行后，should_send_request_的值。如果为false，则表示不应该发送请求，
    // 直接返回FAILURE状态。
    if (!should_send_request_) {
      return BT::NodeStatus::FAILURE;
    }

    // 如果决定发送请求，则使用service_client_发送异步请求。这里的request_是之前准备好的请求数据。
    // async_send_request方法将发送请求，并返回一个future对象，这个对象代表了请求结果的一个未来状态。
    // 使用share方法使得这个future对象可以被安全地复制和共享。
    future_result_ = service_client_->async_send_request(request_).share();
    
    // 记录发送请求的时间。
    sent_time_ = node_->now();
    
    // 设置request_sent_标志为true，表示请求已经发送。
    request_sent_ = true;
  }
  
  // 调用check_future方法检查future_result_的状态，根据请求的处理结果返回相应的节点状态。
  // 这个方法的具体实现在这段代码中没有给出，它应该是用来检查异步请求结果的状态，并据此返回节点的状态。
  return check_future();
}

  /**
   * @brief The other (optional) override required by a BT service.
   */
  void halt() override
  {
    request_sent_ = false;
    setStatus(BT::NodeStatus::IDLE);
  }

  /**
   * @brief Function to perform some user-defined operation on tick
   * Fill in service request with information if necessary
   */
  virtual void on_tick()
  {
  }

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the service. Could put a value on the blackboard.
   * @param response can be used to get the result of the service call in the BT Node.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override to return another value
   */

  /**
 * @brief 在服务成功完成后执行一些用户定义的操作的函数。
 * 可以在黑板上设置一个值。
 * @param response 可用于在行为树节点中获取服务调用的结果。
 * @return BT::NodeStatus 默认返回SUCCESS，用户可以重写以返回其他值
 */
  virtual BT::NodeStatus on_completion(std::shared_ptr<typename ServiceT::Response>/*response*/)
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Check the future and decide the status of BT
   * @return BT::NodeStatus SUCCESS if future complete before timeout, FAILURE otherwise
   */

  /**
 * @brief 检查future并决定行为树的状态
 * @return BT::NodeStatus 如果future在超时前完成则返回SUCCESS，否则返回FAILURE
 */
  virtual BT::NodeStatus check_future()
  {
    auto elapsed = (node_->now() - sent_time_).template to_chrono<std::chrono::milliseconds>();
    auto remaining = server_timeout_ - elapsed;

    if (remaining > std::chrono::milliseconds(0)) {
      auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;

      rclcpp::FutureReturnCode rc;
      rc = callback_group_executor_.spin_until_future_complete(future_result_, timeout);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        request_sent_ = false;
        BT::NodeStatus status = on_completion(future_result_.get());
        return status;
      }

      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        on_wait_for_result();
        elapsed = (node_->now() - sent_time_).template to_chrono<std::chrono::milliseconds>();
        if (elapsed < server_timeout_) {
          return BT::NodeStatus::RUNNING;
        }
      }
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "Node timed out while executing service call to %s.", service_name_.c_str());
    request_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation after a timeout waiting
   * for a result that hasn't been received yet
   */
  virtual void on_wait_for_result()
  {
  }

protected:
  /**
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   */
  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  std::string service_name_, service_node_name_;
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;
  std::shared_ptr<typename ServiceT::Request> request_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds server_timeout_;

  // The timeout value for BT loop execution
  std::chrono::milliseconds bt_loop_duration_;

  // To track the server response when a new request is sent
  std::shared_future<typename ServiceT::Response::SharedPtr> future_result_;
  bool request_sent_{false};
  rclcpp::Time sent_time_;

  // Can be set in on_tick or on_wait_for_result to indicate if a request should be sent.
  bool should_send_request_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_

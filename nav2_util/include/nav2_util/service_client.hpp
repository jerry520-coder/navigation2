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

#ifndef NAV2_UTIL__SERVICE_CLIENT_HPP_
#define NAV2_UTIL__SERVICE_CLIENT_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{

/**
 * @class nav2_util::ServiceClient
 * @brief A simple wrapper on ROS2 services for invoke() and block-style calling
 */

/**
 * @class nav2_util::ServiceClient
 * @brief 一个简单的ROS2服务包装器，用于invoke()和阻塞式调用
 */
template<class ServiceT>
class ServiceClient
{
public:
  /**
  * @brief A constructor
  * @param service_name name of the service to call
  * @param provided_node Node to create the service client off of
  */

 /**
 * @brief 构造函数
 * @param service_name 要调用的服务的名称
 * @param provided_node 用于创建服务客户端的节点
 */
  explicit ServiceClient(
    const std::string & service_name,
    const rclcpp::Node::SharedPtr & provided_node)
  : service_name_(service_name), node_(provided_node)
  {
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    client_ = node_->create_client<ServiceT>(
      service_name,
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      callback_group_);
  }

  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  /**
  * @brief Invoke the service and block until completed or timed out
  * @param request The request object to call the service using
  * @param timeout Maximum timeout to wait for, default infinite
  * @return Response A pointer to the service response from the request
  */

 /**
 * @brief 调用服务并阻塞，直到完成或超时
 * @param request 用于调用服务的请求对象
 * @param timeout 等待的最大超时时间，默认为无限
 * @return Response 指向请求服务响应的指针
//  *  -1 通常用来表示一个无限期的超时或者一个特殊的时间点。
 */
  typename ResponseType::SharedPtr invoke(
    typename RequestType::SharedPtr & request,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1))
  {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        node_->get_logger(), "%s service client: waiting for service to appear...",
        service_name_.c_str());
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "%s service client: send async request",
      service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    if (callback_group_executor_.spin_until_future_complete(future_result, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // Pending request must be manually cleaned up if execution is interrupted or timed out
      client_->remove_pending_request(future_result);
      throw std::runtime_error(service_name_ + " service client: async_send_request failed");
    }

    return future_result.get();
  }

  /**
  * @brief Invoke the service and block until completed
  * @param request The request object to call the service using
  * @param Response A pointer to the service response from the request
  * @return bool Whether it was successfully called
  */

 /**
 * @brief 调用服务并阻塞，直到完成
 * @param request 用于调用服务的请求对象
 * @param Response 从请求中得到的服务响应的指针
 * @return bool 是否成功调用
 */
  bool invoke(
    typename RequestType::SharedPtr & request,
    typename ResponseType::SharedPtr & response)
  {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
                service_name_ + " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(
        node_->get_logger(), "%s service client: waiting for service to appear...",
        service_name_.c_str());
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "%s service client: send async request",
      service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    if (callback_group_executor_.spin_until_future_complete(future_result) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // Pending request must be manually cleaned up if execution is interrupted or timed out
      client_->remove_pending_request(future_result);
      return false;
    }

    response = future_result.get();
    return response.get();
  }

  /**
  * @brief Block until a service is available or timeout
  * @param timeout Maximum timeout to wait for, default infinite
  * @return bool true if service is available
  */

 /**
 * @brief 阻塞直到服务可用或超时
 * @param timeout 等待的最大超时时间，默认为无限
 * @return bool 如果服务可用，则返回true
 */
  bool wait_for_service(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max())
  {
    return client_->wait_for_service(timeout);
  }

  /**
  * @brief Gets the service name
  * @return string Service name
  */
  std::string getServiceName()
  {
    return service_name_;
  }

protected:
  std::string service_name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__SERVICE_CLIENT_HPP_

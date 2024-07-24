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

#ifndef NAV2_UTIL__LIFECYCLE_UTILS_HPP_
#define NAV2_UTIL__LIFECYCLE_UTILS_HPP_

#include <vector>
#include <string>
#include <chrono>
#include "nav2_util/string_utils.hpp"

namespace nav2_util
{

/// Transition the given lifecycle nodes to the ACTIVATED state in order
/** At this time, service calls frequently hang for unknown reasons. The only
 *  way to combat that is to timeout the service call and retry it. To use this
 *  function, estimate how long your nodes should take to at each transition and
 *  set your timeout accordingly.
 * \param[in] node_names A vector of the fully qualified node names to startup.
 * \param[in] service_call_timeout The maximum amount of time to wait for a
 *            service call.
 * \param[in] retries The number of times to try a state transition service call
 */

/// 按顺序将给定的生命周期节点转换到激活状态ACTIVATED
/** 目前，服务调用经常因未知原因而挂起。解决这个问题的唯一方法是设置服务调用的超时并重试。要使用此功能，
 *  请估计节点在每个转换阶段应该花费的时间，并相应地设置超时时间。
 * \param[in] node_names 要启动的完全限定的节点名称的向量。
 * \param[in] service_call_timeout 等待服务调用的最大时间。
 * \param[in] retries 尝试状态转换服务调用的次数
 */
void startup_lifecycle_nodes(
  const std::vector<std::string> & node_names,
  const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
  const int retries = 3);

/// Transition the given lifecycle nodes to the ACTIVATED state in order.
/**
 * \param[in] nodes A ':' seperated list of node names. eg. "/node1:/node2"
 */

/// 按顺序将给定的生命周期节点转换到激活状态。
/**
 * \param[in] nodes  A ':' 节点名称的冒号分隔列表。例如："/node1:/node2"
 */
void startup_lifecycle_nodes(
  const std::string & nodes,
  const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
  const int retries = 3)
{
  startup_lifecycle_nodes(split(nodes, ':'), service_call_timeout, retries);
}

/// Transition the given lifecycle nodes to the UNCONFIGURED state in order
/** At this time, service calls frequently hang for unknown reasons. The only
 *  way to combat that is to timeout the service call and retry it. To use this
 *  function, estimate how long your nodes should take to at each transition and
 *  set your timeout accordingly.
 * \param[in] node_names A vector of the fully qualified node names to reset.
 * \param[in] service_call_timeout The maximum amount of time to wait for a
 *            service call.
 * \param[in] retries The number of times to try a state transition service call
 */

/// 按顺序将给定的生命周期节点转换到未配置状态UNCONFIGURED
/** 目前，服务调用经常因未知原因而挂起。解决这个问题的唯一方法是设置服务调用的超时并重试。要使用此功能，
 *  请估计节点在每个转换阶段应该花费的时间，并相应地设置超时时间。
 * \param[in] node_names 要重置的完全限定的节点名称的向量。
 * \param[in] service_call_timeout 等待服务调用的最大时间。
 * \param[in] retries 尝试状态转换服务调用的次数
 */
void reset_lifecycle_nodes(
  const std::vector<std::string> & node_names,
  const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
  const int retries = 3);

/// Transition the given lifecycle nodes to the UNCONFIGURED state in order.
/**
 * \param[in] nodes A ':' seperated list of node names. eg. "/node1:/node2"
 */

/// 按顺序将给定的生命周期节点转换到未配置状态。
/**
 * \param[in] nodes 节点名称的冒号分隔列表。例如："/node1:/node2"
 */
void reset_lifecycle_nodes(
  const std::string & nodes,
  const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
  const int retries = 3)
{
  reset_lifecycle_nodes(split(nodes, ':'), service_call_timeout, retries);
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_UTILS_HPP_

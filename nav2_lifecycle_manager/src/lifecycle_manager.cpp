// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2022 Samsung Research America
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

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::msg::State;
using nav2_util::LifecycleServiceClient;

namespace nav2_lifecycle_manager
{

LifecycleManager::LifecycleManager(const rclcpp::NodeOptions & options)
: Node("lifecycle_manager", options), diagnostics_updater_(this)
{
  // 创建时记录信息
  RCLCPP_INFO(get_logger(), "Creating");

  // The list of names is parameterized, allowing this module to be used with a different set
  // of nodes
  // 声明参数。这些参数允许对使用的节点名单、自动启动设置、绑定超时和重连尝试等进行配置
  declare_parameter("node_names", rclcpp::PARAMETER_STRING_ARRAY);
  declare_parameter("autostart", rclcpp::ParameterValue(false));
  declare_parameter("bond_timeout", 4.0);
  declare_parameter("bond_respawn_max_duration", 10.0);
  declare_parameter("attempt_respawn_reconnection", true);

 // 注册一个在ROS 2环境准备关闭前执行的回调函数 
  registerRclPreshutdownCallback();

// 从参数服务器获取参数值，并赋值给成员变量
  node_names_ = get_parameter("node_names").as_string_array();
  get_parameter("autostart", autostart_);
  double bond_timeout_s;
  get_parameter("bond_timeout", bond_timeout_s);
  bond_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(bond_timeout_s));

  double respawn_timeout_s;
  get_parameter("bond_respawn_max_duration", respawn_timeout_s);
  bond_respawn_max_duration_ = rclcpp::Duration::from_seconds(respawn_timeout_s);

  get_parameter("attempt_respawn_reconnection", attempt_respawn_reconnection_);

 // 创建一个回调组，用于管理节点服务调用的互斥性
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

   // 创建服务，用于管理生命周期节点
  manager_srv_ = create_service<ManageLifecycleNodes>(
    get_name() + std::string("/manage_nodes"),
    std::bind(&LifecycleManager::managerCallback, this, _1, _2, _3),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);

 // 创建服务，用于检查节点是否激活
  is_active_srv_ = create_service<std_srvs::srv::Trigger>(
    get_name() + std::string("/is_active"),
    std::bind(&LifecycleManager::isActiveCallback, this, _1, _2, _3),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);

// 初始化状态映射，用于处理不同生命周期状态的转换
  transition_state_map_[Transition::TRANSITION_CONFIGURE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_CLEANUP] = State::PRIMARY_STATE_UNCONFIGURED;
  transition_state_map_[Transition::TRANSITION_ACTIVATE] = State::PRIMARY_STATE_ACTIVE;
  transition_state_map_[Transition::TRANSITION_DEACTIVATE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    State::PRIMARY_STATE_FINALIZED;

 // 初始化状态转换时的标签映射，用于日志记录和用户界面显示
  transition_label_map_[Transition::TRANSITION_CONFIGURE] = std::string("Configuring ");
  transition_label_map_[Transition::TRANSITION_CLEANUP] = std::string("Cleaning up ");
  transition_label_map_[Transition::TRANSITION_ACTIVATE] = std::string("Activating ");
  transition_label_map_[Transition::TRANSITION_DEACTIVATE] = std::string("Deactivating ");
  transition_label_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    std::string("Shutting down ");

  // 创建一个定时器，用于初始化生命周期服务客户端
  init_timer_ = this->create_wall_timer(
    0s,  // 定时器触发时间，0秒表示立即触发
    [this]() -> void {  // 定时器触发时执行的lambda表达式
      init_timer_->cancel();  // 取消当前定时器，防止重复触发
      createLifecycleServiceClients();  // 创建生命周期服务客户端

      if (autostart_) {  // 检查是否设置了自动启动
        // 如果设置了自动启动，再次创建一个定时器，用于启动所有节点
        init_timer_ = this->create_wall_timer(
          0s,  // 同样设置为0秒，即立即触发
          [this]() -> void {
            init_timer_->cancel();  // 取消当前定时器
            startup();  // 启动所有节点的函数
          },
          callback_group_);  // 使用特定的回调组
      }

      // 创建一个单线程执行器
      auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor->add_callback_group(callback_group_, get_node_base_interface());  // 将回调组添加到执行器
      service_thread_ = std::make_unique<nav2_util::NodeThread>(executor);  // 在新线程中启动执行器
    });  

// 设置硬件ID为"Nav2"，用于诊断信息中标识硬件或软件实体
  diagnostics_updater_.setHardwareID("Nav2");

  // 添加一个诊断任务
  diagnostics_updater_.add("Nav2 Health", this, &LifecycleManager::CreateActiveDiagnostic);
}

LifecycleManager::~LifecycleManager()
{
  RCLCPP_INFO(get_logger(), "Destroying %s", get_name());
  service_thread_.reset();
}

void
LifecycleManager::managerCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ManageLifecycleNodes::Request> request,
  std::shared_ptr<ManageLifecycleNodes::Response> response)
{
  switch (request->command) {
    case ManageLifecycleNodes::Request::STARTUP:
      response->success = startup();
      break;
    case ManageLifecycleNodes::Request::RESET:
      response->success = reset();
      break;
    case ManageLifecycleNodes::Request::SHUTDOWN:
      response->success = shutdown();
      break;
    case ManageLifecycleNodes::Request::PAUSE:
      response->success = pause();
      break;
    case ManageLifecycleNodes::Request::RESUME:
      response->success = resume();
      break;
  }
}

void
LifecycleManager::isActiveCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = system_active_;
}

void
LifecycleManager::CreateActiveDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (system_active_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nav2 is active");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Nav2 is inactive");
  }
}

void
LifecycleManager::createLifecycleServiceClients()
{
  message("Creating and initializing lifecycle service clients");
  for (auto & node_name : node_names_) {
    node_map_[node_name] =
      std::make_shared<LifecycleServiceClient>(node_name, shared_from_this());
  }
}

void
LifecycleManager::destroyLifecycleServiceClients()
{
  message("Destroying lifecycle service clients");
  for (auto & kv : node_map_) {
    kv.second.reset();
  }
}

bool
LifecycleManager::createBondConnection(const std::string & node_name)
{
  // 将存储的超时时间从毫秒转换为纳秒，然后计算出秒数
  const double timeout_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(bond_timeout_).count();
  const double timeout_s = timeout_ns / 1e9; // 将纳秒转换为秒

  // 检查指定的节点是否已经存在于bond映射中并且超时时间大于0
  if (bond_map_.find(node_name) == bond_map_.end() && bond_timeout_.count() > 0.0) {
    // 如果节点不在绑定映射中，创建一个新的bond实例，并加入映射
    bond_map_[node_name] =
      std::make_shared<bond::Bond>("bond", node_name, shared_from_this());
    // 设置心跳超时时间
    bond_map_[node_name]->setHeartbeatTimeout(timeout_s);
    // 设置心跳周期
    bond_map_[node_name]->setHeartbeatPeriod(0.10);
    // 开始bond
    bond_map_[node_name]->start();

    // 等待bond形成，超时时间为原本超时时间的一半
    if (
      !bond_map_[node_name]->waitUntilFormed(
        rclcpp::Duration(rclcpp::Duration::from_nanoseconds(timeout_ns / 2))))
    {
      // 如果绑定没有在超时时间内成功形成，记录错误日志并返回false
      RCLCPP_ERROR(
        get_logger(),
        "Server %s was unable to be reached after %0.2fs by bond. "
        "This server may be misconfigured.",
        node_name.c_str(), timeout_s);
      return false;
    }
    // 如果绑定成功形成，记录信息日志
    RCLCPP_INFO(get_logger(), "Server %s connected with bond.", node_name.c_str());
  }

  // 返回true表示绑定成功
  return true;
}

bool
LifecycleManager::changeStateForNode(const std::string & node_name, std::uint8_t transition)
{
  message(transition_label_map_[transition] + node_name);

  if (!node_map_[node_name]->change_state(transition) ||
    !(node_map_[node_name]->get_state() == transition_state_map_[transition]))
  {
    RCLCPP_ERROR(get_logger(), "Failed to change state for node: %s", node_name.c_str());
    return false;
  }

  if (transition == Transition::TRANSITION_ACTIVATE) {
    return createBondConnection(node_name);
  } else if (transition == Transition::TRANSITION_DEACTIVATE) {
    bond_map_.erase(node_name);
  }

  return true;
}

bool
LifecycleManager::changeStateForAllNodes(std::uint8_t transition, bool hard_change)
{
  // Hard change will continue even if a node fails
  if (transition == Transition::TRANSITION_CONFIGURE ||
    transition == Transition::TRANSITION_ACTIVATE)
  {
    for (auto & node_name : node_names_) {
      try {
        if (!changeStateForNode(node_name, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to change state for node: %s. Exception: %s.", node_name.c_str(), e.what());
        return false;
      }
    }
  } else {
    std::vector<std::string>::reverse_iterator rit;
    for (rit = node_names_.rbegin(); rit != node_names_.rend(); ++rit) {
      try {
        if (!changeStateForNode(*rit, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to change state for node: %s. Exception: %s.", (*rit).c_str(), e.what());
        return false;
      }
    }
  }
  return true;
}

void
LifecycleManager::shutdownAllNodes()
{
  message("Deactivate, cleanup, and shutdown nodes");
  changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE);
  changeStateForAllNodes(Transition::TRANSITION_CLEANUP);
  changeStateForAllNodes(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

bool
LifecycleManager::startup()
{
    // 在日志中输出启动节点的信息
    message("Starting managed nodes bringup...");

    // 尝试将所有节点配置到配置状态，然后激活它们。
    // 如果任一步骤失败，记录错误并返回false
    if (!changeStateForAllNodes(Transition::TRANSITION_CONFIGURE) ||
        !changeStateForAllNodes(Transition::TRANSITION_ACTIVATE))
    {
      // 如果无法成功启动所有节点，则在日志中记录错误并中止启动过程
      RCLCPP_ERROR(get_logger(), "Failed to bring up all requested nodes. Aborting bringup.");
      return false;
    }

    // 如果所有节点均成功启动，输出节点激活的信息
    message("Managed nodes are active");

    // 设置系统激活标志为真，表示所有管理的节点均已成功激活
    system_active_ = true;

    // 创建bond计时器，用于监视节点之间的连接状态
    createBondTimer();

    // 返回true表示所有节点已成功启动并激活
    return true;
}

bool
LifecycleManager::shutdown()
{
  system_active_ = false;
  destroyBondTimer();

  message("Shutting down managed nodes...");
  shutdownAllNodes();
  destroyLifecycleServiceClients();
  message("Managed nodes have been shut down");
  return true;
}

bool
LifecycleManager::reset(bool hard_reset)
{
  system_active_ = false;
  destroyBondTimer();

  message("Resetting managed nodes...");
  // Should transition in reverse order
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE, hard_reset) ||
    !changeStateForAllNodes(Transition::TRANSITION_CLEANUP, hard_reset))
  {
    if (!hard_reset) {
      RCLCPP_ERROR(get_logger(), "Failed to reset nodes: aborting reset");
      return false;
    }
  }

  message("Managed nodes have been reset");
  return true;
}

bool
LifecycleManager::pause()
{
  system_active_ = false;
  destroyBondTimer();

  message("Pausing managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to pause nodes: aborting pause");
    return false;
  }

  message("Managed nodes have been paused");
  return true;
}

bool
LifecycleManager::resume()
{
  message("Resuming managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_ACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to resume nodes: aborting resume");
    return false;
  }

  message("Managed nodes are active");
  system_active_ = true;
  createBondTimer();
  return true;
}

void
LifecycleManager::createBondTimer()
{
  if (bond_timeout_.count() <= 0) {
    return;
  }

  message("Creating bond timer...");
  bond_timer_ = this->create_wall_timer(
    200ms,
    std::bind(&LifecycleManager::checkBondConnections, this),
    callback_group_);
}

void
LifecycleManager::destroyBondTimer()
{
  if (bond_timer_) {
    message("Terminating bond timer...");
    bond_timer_->cancel();
    bond_timer_.reset();
  }
}

void
LifecycleManager::onRclPreshutdown()
{
  RCLCPP_INFO(
    get_logger(), "Running Nav2 LifecycleManager rcl preshutdown (%s)",
    this->get_name());

  destroyBondTimer();

  /*
   * Dropping the bond map is what we really need here, but we drop the others
   * to prevent the bond map being used. Likewise, squash the service thread.
   */
  service_thread_.reset();
  node_names_.clear();
  node_map_.clear();
  bond_map_.clear();
}

void
LifecycleManager::registerRclPreshutdownCallback()
{
  rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

  context->add_pre_shutdown_callback(
    std::bind(&LifecycleManager::onRclPreshutdown, this)
  );
}

void
LifecycleManager::checkBondConnections()
{
  if (!system_active_ || !rclcpp::ok() || bond_map_.empty()) {
    return;
  }

  for (auto & node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    if (bond_map_[node_name]->isBroken()) {
      message(
        std::string(
          "Have not received a heartbeat from " + node_name + "."));

      // if one is down, bring them all down
      RCLCPP_ERROR(
        get_logger(),
        "CRITICAL FAILURE: SERVER %s IS DOWN after not receiving a heartbeat for %i ms."
        " Shutting down related nodes.",
        node_name.c_str(), static_cast<int>(bond_timeout_.count()));
      reset(true);  // hard reset to transition all still active down
      // if a server crashed, it won't get cleared due to failed transition, clear manually
      bond_map_.clear();

      // Initialize the bond respawn timer to check if server comes back online
      // after a failure, within a maximum timeout period.
      if (attempt_respawn_reconnection_) {
        bond_respawn_timer_ = this->create_wall_timer(
          1s,
          std::bind(&LifecycleManager::checkBondRespawnConnection, this),
          callback_group_);
      }
      return;
    }
  }
}

void
LifecycleManager::checkBondRespawnConnection()
{
  // First attempt in respawn, start maximum duration to respawn
  if (bond_respawn_start_time_.nanoseconds() == 0) {
    bond_respawn_start_time_ = now();
  }

  // Note: system_active_ is inverted since this should be in a failure
  // condition. If another outside user actives the system again, this should not process.
  if (system_active_ || !rclcpp::ok() || node_names_.empty()) {
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
    return;
  }

  // Check number of live connections after a bond failure
  int live_servers = 0;
  const int max_live_servers = node_names_.size();
  for (auto & node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    try {
      node_map_[node_name]->get_state();  // Only won't throw if the server exists
      live_servers++;
    } catch (...) {
      break;
    }
  }

  // If all are alive, kill timer and retransition system to active
  // Else, check if maximum timeout has occurred
  if (live_servers == max_live_servers) {
    message("Successfully re-established connections from server respawns, starting back up.");
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
    startup();
  } else if (now() - bond_respawn_start_time_ >= bond_respawn_max_duration_) {
    message("Failed to re-establish connection from a server crash after maximum timeout.");
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
  }
}

#define ANSI_COLOR_RESET    "\x1b[0m" // 重置颜色到默认
#define ANSI_COLOR_BLUE     "\x1b[34m" // 设置颜色为蓝色

void
LifecycleManager::message(const std::string & msg)
{
    // 使用RCLCPP_INFO宏将消息输出到ROS 2日志系统，消息文本使用蓝色和加粗格式
  // '\33[1m' 和 '\33[0m' 分别是ANSI控制码，用于设置文本为加粗和重置所有属性到默认
  RCLCPP_INFO(get_logger(), ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
}

}  // namespace nav2_lifecycle_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_lifecycle_manager::LifecycleManager)

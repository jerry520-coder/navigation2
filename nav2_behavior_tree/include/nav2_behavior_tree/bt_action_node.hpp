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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_behavior_tree
{

using namespace std::chrono_literals;  // NOLINT 计时器字面词

/**
 * @brief 表示基于动作的 BT 节点的抽象类。Abstract class representing an action based BT node
 * @tparam ActionT Type of action
 */
template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::BtActionNode constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */

/**
 * @brief BtActionNode的构造函数
 * @param xml_tag_name 该节点在XML中的标签名称
 * @param action_name 该节点创建客户端要交互的动作名称
 * @param conf 行为树节点的配置
 */
BtActionNode(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name), should_send_goal_(true)
{
  // 从配置的黑板中获取节点的共享指针
  // 使用 template 关键字是因为 get 函数是一个模板函数，需要明确指出其模板参数类型。
  node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node"); //通常出现在调用依赖于模板参数的成员模板函数时，特别是当编译器可能有歧义不清楚该名称表示一个模板还是一个普通成员时。
  
  // 为节点创建一个回调组，设置为互斥类型，禁用自动添加到执行器
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  
  // 将回调组添加到执行器中，用于执行回调函数
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 从黑板中获取行为树循环持续时间和服务器超时时间，并初始化
  bt_loop_duration_ =
    config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
  server_timeout_ =
    config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  
  // 尝试从输入中获取服务器超时时间，并更新
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

  // 初始化目标和结果消息
  //是从ActionT动作类型中创建一个Goal类型的实例
  //Goal() 和 WrappedResult() 看起来像是类型的构造函数
  goal_ = typename ActionT::Goal(); 
  result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();
  // `typename` 关键字用于指明 `ActionT::Goal` 和 `rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult` 是在模板 `ActionT` 中定义的类型。

  // 尝试从输入中获取重映射的动作名称，并更新
  std::string remapped_action_name;
  if (getInput("server_name", remapped_action_name)) {
    action_name_ = remapped_action_name;
  }
  
  // 创建动作客户端
  createActionClient(action_name_);


  RCLCPP_DEBUG(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
}

  BtActionNode() = delete; //显式地删除这个默认构造函数

  virtual ~BtActionNode()
  {
  }

  /**
   * @brief Create instance of an action client
   * @param action_name Action name to create client for
   */

/**
   * @brief 创建动作客户端的实例
   * @param action_name 要为其创建客户端的动作名称
   */
  void createActionClient(const std::string & action_name)
  {
    // 现在我们已经有了ROS节点，用这个BT（行为树）动作创建动作客户端
    // 这里使用了rclcpp_action命名空间下的create_client方法来创建一个动作客户端
    // 参数包括：node_（ROS节点），action_name（动作名称），callback_group_（回调组，用于管理回调函数的执行）
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

    // 在继续之前确保服务器实际上是存在的
    // 这里使用了ROS的调试宏RCLCPP_DEBUG向日志输出信息，表示正在等待动作服务器
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());

    // 使用action_client_的wait_for_action_server方法等待动作服务器，这里的等待时间设置为1秒（1s）
    // 如果在1秒内未发现动作服务器，则执行if语句块中的内容
    if (!action_client_->wait_for_action_server(1s)) {
      // 使用RCLCPP_ERROR向日志输出错误信息，表示在等待1秒后未找到动作服务器
      RCLCPP_ERROR(
        node_->get_logger(), "\"%s\" action server not available after waiting for 1 s",
        action_name.c_str());

      // 抛出一个运行时错误，错误信息指出动作服务器不可用
      throw std::runtime_error(
              std::string("Action server ") + action_name +
              std::string(" not available"));
    }
  }

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */

  /**
 * @brief 任何接受参数的 BtActionNode 的子类都必须提供一个 providedPorts 方法，并在其中调用 providedBasicPorts。
 * @param addition 要添加到 BT 端口列表中的额外端口
 * @return BT::PortsList 包含基本端口及节点特定端口
 */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */

  /**
 * @brief 创建 BT 端口列表
 * @return BT::PortsList 包含基本端口及节点特定端口
 */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success
  // 派生类可以重写以下任何方法，以便在动作的处理中加入钩子：on_tick、on_wait_for_result 和 on_success


  /**
   * @brief Function to perform some user-defined operation on tick
   * Could do dynamic checks, such as getting updates to values on the blackboard
   */

  /**
 * @brief 执行某些用户定义的操作的函数，在 tick 上
 * 可以执行动态检查，例如获取黑板上值的更新
 */
  virtual void on_tick()
  {
  }

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet. Also provides access to
   * the latest feedback message from the action server. Feedback will be nullptr
   * in subsequent calls to this function if no new feedback is received while waiting for a result.
   * @param feedback shared_ptr to latest feedback message, nullptr if no new feedback was received
   */

  /**
 * @brief 在等待未收到的结果超时后执行某些用户定义的操作的函数。还提供对动作服务器的最新反馈消息的访问。
 * 如果在等待结果时没有收到新的反馈，那么在后续调用此函数时，反馈将是 nullptr。
 * @param feedback 指向最新反馈消息的 shared_ptr，如果没有收到新的反馈则为 nullptr
 */
  virtual void on_wait_for_result(std::shared_ptr<const typename ActionT::Feedback>/*feedback*/)
  {
  }

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the action. Could put a value on the blackboard.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */

  /**
 * @brief 在动作成功完成后执行一些用户定义的操作的函数。可以在黑板上放置一个值。
 * @return BT::NodeStatus 默认返回 SUCCESS，用户可以覆盖返回另一个值
 */
  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Function to perform some user-defined operation when the action is aborted.
   * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
   */
  
  /**
 * @brief 当动作被中止时执行一些用户定义的操作的函数。
 * @return BT::NodeStatus 默认返回 FAILURE，用户可以覆盖返回另一个值
 */
  virtual BT::NodeStatus on_aborted()
  {
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation when the action is cancelled.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */

  /**
 * @brief 当动作被取消时执行一些用户定义的操作的函数。
 * @return BT::NodeStatus 默认返回 SUCCESS，用户可以覆盖返回另一个值
 */
  virtual BT::NodeStatus on_cancelled()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */

/**
 * @brief 由 BT 动作需要的主要重写
 * @return BT::NodeStatus tick 执行的状态
 * @note 主要用于在行为树执行过程中处理与动作服务器的交互
 */
  BT::NodeStatus tick() override
  {
    // 如果节点状态是IDLE，表示这是动作的开始
    if (status() == BT::NodeStatus::IDLE) {
      // 将节点状态设置为RUNNING，以通知行为树的日志记录系统（如果有）
      setStatus(BT::NodeStatus::RUNNING);

      // 重置标志位，决定是否发送目标，允许用户在on_tick中进行设置
      should_send_goal_ = true;

      // 用户定义的回调函数，可能会修改"should_send_goal_"的值
      on_tick();

      // 如果用户决定不发送目标，则直接返回失败状态
      if (!should_send_goal_) {
        return BT::NodeStatus::FAILURE;
      }
      // 发送新的目标
      send_new_goal();
    }

    // 尝试检查目标的状态
    try {
      // 如果已经发送了新目标，但动作服务器尚未响应
      // 检查未来目标句柄（future_goal_handle_）
      if (future_goal_handle_) {
        auto elapsed = (node_->now() - time_goal_sent_).template to_chrono<std::chrono::milliseconds>(); //历时
        // 检查目标是否完成，如果没有完成且未超时，则返回RUNNING状态
        if (!is_future_goal_handle_complete(elapsed)) {
        // return RUNNING if there is still some time before timeout happens
          if (elapsed < server_timeout_) {
            return BT::NodeStatus::RUNNING;
          }
          // 如果服务器响应时间超过了设定的超时时间，则返回失败
          RCLCPP_WARN(
            node_->get_logger(),
            "Timed out while waiting for action server to acknowledge goal request for %s",
            action_name_.c_str());
          future_goal_handle_.reset();
          return BT::NodeStatus::FAILURE;
        }
      }

      // 下面的代码对应于"RUNNING"状态的循环
      if (rclcpp::ok() && !goal_result_available_) {
        // 用户定义的回调，可能会修改"goal_updated_"的值
        on_wait_for_result(feedback_);

        // 重置反馈以避免陈旧的信息
        feedback_.reset();

        auto goal_status = goal_handle_->get_status();
        // 如果目标被更新且目标状态为执行中或已接受，发送新目标
        if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
        {
          goal_updated_ = false;
          send_new_goal();
          auto elapsed = (node_->now() - time_goal_sent_).template to_chrono<std::chrono::milliseconds>();
          if (!is_future_goal_handle_complete(elapsed)) {
            if (elapsed < server_timeout_) {
              return BT::NodeStatus::RUNNING;
            }
            RCLCPP_WARN(
              node_->get_logger(),
              "Timed out while waiting for action server to acknowledge goal request for %s",
              action_name_.c_str());
            future_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;
          }
        }

        // 调用spin_some()来处理回调
        callback_group_executor_.spin_some();

        // 检查是否在调用spin_some()后最终收到了结果
        if (!goal_result_available_) {
          // 如果还没有结果，继续返回RUNNING状态
          return BT::NodeStatus::RUNNING;
        }
      }
    } catch (const std::runtime_error & e) {
      // 处理发送目标失败的情况，返回失败状态
      if (e.what() == std::string("send_goal failed") ||
        e.what() == std::string("Goal was rejected by the action server"))
      {
      // Action related failure that should not fail the tree, but the node
        return BT::NodeStatus::FAILURE;
      } else {
        // 内部异常应该被传播到行为树
        throw e;
      }
    }

    BT::NodeStatus status;
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        status = on_success(); //可以用户指定增加其他成功条件
        break;

      case rclcpp_action::ResultCode::ABORTED:
        status = on_aborted();//可以用户指定增加其他中断条件
        break;

      case rclcpp_action::ResultCode::CANCELED:
        status = on_cancelled();//可以用户指定增加其他取消条件
        break;

      default:
        throw std::logic_error("BtActionNode::Tick: invalid status value");
    }

    goal_handle_.reset();
    return status;
  }

  /**
   * @brief The other (optional) override required by a BT action. In this case, we
   * make sure to cancel the ROS2 action if it is still running.
   */

  /**
 * @brief BT 动作需要的另一个（可选的）重写。在这种情况下，我们确保如果 ROS2 动作仍在运行，则取消它。
 * @note 作用是在特定条件下取消与某个动作服务器的目标并获取结果。
 */
void halt() override
{
  // 首先检查是否应该取消当前的目标。
  if (should_cancel_goal()) {
    // 如果需要取消目标，首先获取当前目标的结果。这个操作是异步的，返回一个future对象。
    auto future_result = action_client_->async_get_result(goal_handle_);
    // 然后，尝试取消当前目标，这个操作也是异步的，同样返回一个future对象。
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);

    // 使用callback_group_executor_来等待取消操作完成。如果在指定的server_timeout_时间内未完成，则记录一条错误日志。
    if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      // 如果未能成功取消，记录错误信息到日志。
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to cancel action server for %s", action_name_.c_str());
    }

    // 使用callback_group_executor_等待获取结果操作完成。如果在指定的server_timeout_时间内未完成，则记录一条错误日志。
    if (callback_group_executor_.spin_until_future_complete(future_result, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      // 如果未能成功获取结果，记录错误信息到日志。
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to get result for %s in node halt!", action_name_.c_str());
    }

    // 当取消操作完成后，调用on_cancelled函数。
    on_cancelled();
  }

  // 设置当前节点的状态为IDLE，表示当前节点处于空闲状态。
  setStatus(BT::NodeStatus::IDLE);
}

protected:
  /**
   * @brief Function to check if current goal should be cancelled
   * @return bool True if current goal should be cancelled, false otherwise
   */

  /**
 * @brief 检查当前目标是否应该被取消的函数
 * @return bool 如果当前目标应该被取消则返回 true，否则返回 false
 * @note 用来判断是否应该取消当前的目标
 */
bool should_cancel_goal()
{
  // 首先判断当前节点的状态。如果节点的状态不是RUNNING，那么就没有必要取消目标，因为节点并未在执行任务。
  // 在这里，status()函数应该是用来获取当前节点的状态。
  if (status() != BT::NodeStatus::RUNNING) {
    // 如果节点不是运行状态，则返回false，表示不需要取消目标。
    return false;
  }

  // 接下来检查目标句柄（goal_handle_）是否有效。如果目标句柄无效，同样没有取消目标的必要。
  if (!goal_handle_) {
    // 如果目标句柄无效，则返回false，表示不需要取消目标。
    return false;
  }

  // 执行一些回调处理，可能是为了处理累积的回调事件。
  callback_group_executor_.spin_some();

  // 获取当前目标的状态。这里使用了目标句柄的get_status函数来获取状态。
  auto status = goal_handle_->get_status();

  // 最后判断目标的状态。如果目标状态是STATUS_ACCEPTED（目标已被接受但尚未执行）或
  // STATUS_EXECUTING（目标正在执行中），则应该取消目标。
  // 在这里，action_msgs::msg::GoalStatus是一个枚举类型，包含了可能的目标状态。
  return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
}

  /**
   * @brief Function to send new goal to action server
   */

  /**
 * @brief 向动作服务器发送新目标的函数
 * @note 作用是发送一个新的目标给动作服务器，并设置目标结果与反馈的回调函数
 */
void send_new_goal()
{
  // 首先将目标结果可用的标志设置为false，表示新的目标尚未有结果。
  goal_result_available_ = false;

  // 初始化发送目标的选项
  auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();

  // 设置结果回调函数。当目标的结果可用时，这个回调函数会被调用。
  send_goal_options.result_callback =
    [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) {
      // 检查是否有未处理的目标句柄。如果有，可能是因为还没有收到目标的响应，这种情况可能是上一个目标的结果。
      if (future_goal_handle_) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "Goal result for %s available, but it hasn't received the goal response yet. "
          "It's probably a goal result for the last goal request", action_name_.c_str());
        return;
      }

      // TODO注释说明这是一个临时解决方案，等待rcl_action接口更新。
      // 如果目标ID不匹配，则忽略此结果，因为它可能是旧目标的回调结果。
      // 如果匹配，那么必须处理此结果，包括处理目标被中断的情况。
      if (this->goal_handle_->get_goal_id() == result.goal_id) {
        goal_result_available_ = true; // 标记目标结果为可用。
        result_ = result; // 保存目标的结果。
      }
    };

  // 设置反馈回调函数。当收到目标的反馈时，这个回调函数会被调用。
  send_goal_options.feedback_callback =
    [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
      const std::shared_ptr<const typename ActionT::Feedback> feedback) {
      feedback_ = feedback; // 保存目标的反馈。
    };

  // 发送目标，并将返回的future对象保存在future_goal_handle_中。
  // 这个操作是异步的，future对象表示未来某时刻目标句柄的状态。
  future_goal_handle_ = std::make_shared<
    std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>(
    action_client_->async_send_goal(goal_, send_goal_options));

  // 记录发送目标的时间。
  time_goal_sent_ = node_->now();
}

  /**
   * @brief Function to check if the action server acknowledged a new goal
   * @param elapsed Duration since the last goal was sent and future goal handle has not completed.
   * After waiting for the future to complete, this value is incremented with the timeout value.
   * @return boolean True if future_goal_handle_ returns SUCCESS, False otherwise
   */

  /**
 * @brief 检查动作服务器是否已确认新目标的函数
 * @param elapsed 自上一个目标发送以来经过的时间，且未来目标句柄尚未完成。
 * 在等待未来完成之后，此值会增加超时值。
 * @return boolean 如果 future_goal_handle_ 返回 SUCCESS，则为 True，否则为 False
 */
bool is_future_goal_handle_complete(std::chrono::milliseconds & elapsed)
{
  // 计算从发送目标到现在已经过去的时间与服务器超时时间之差，得到剩余时间
  auto remaining = server_timeout_ - elapsed;

  // 如果服务器已经超时，则没有必要等待，重置未来目标句柄并返回false
  if (remaining <= std::chrono::milliseconds(0)) {
    future_goal_handle_.reset();
    return false;
  }

  // 如果剩余时间大于行为树循环的持续时间，则等待时间设置为行为树循环的持续时间，否则设置为剩余时间
  auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;
  // 等待未来目标句柄完成或超时，等待时间为上面计算的timeout
  auto result =
    callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timeout);
  // 更新已经过去的时间
  elapsed += timeout;

  // 如果等待被中断，重置未来目标句柄并抛出异常
  if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
    future_goal_handle_.reset();
    throw std::runtime_error("send_goal failed");
  }

  // 如果未来目标句柄成功完成，获取目标句柄，重置未来目标句柄
  // 如果获取的目标句柄为空，则表示目标被动作服务器拒绝，抛出异常
  if (result == rclcpp::FutureReturnCode::SUCCESS) {
    goal_handle_ = future_goal_handle_->get();
    future_goal_handle_.reset();
    if (!goal_handle_) {
      throw std::runtime_error("Goal was rejected by the action server");
    }
    // 返回true表示未来目标句柄已经完成
    return true;
  }

  // 如果上述条件都不满足，则返回false表示未来目标句柄尚未完成
  return false;
}

  /**
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   */

  /**
 * @brief 如果此节点包装了一个恢复动作，则在黑板上增加恢复计数的函数
 */
void increment_recovery_count()
{
  // 初始化一个局部变量recovery_count，用于存储从黑板中获取的恢复次数
  int recovery_count = 0;
  
  // 从配置的黑板中获取键为"number_recoveries"的值，并存储到recovery_count变量中
  // 如果黑板上没有这个键，recovery_count的值将保持为0
  // NOLINT是一个lint工具的注释，用来告诉代码检查工具忽略接下来的代码行
  config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
  
  // 将recovery_count的值增加1，代表新增了一次恢复操作
  recovery_count += 1;
  
  // 将更新后的recovery_count值设置回黑板中，键仍然是"number_recoveries"
  // 这样，其他可以访问黑板的行为树节点就可以使用最新的恢复次数了
  config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
}

  std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // All ROS2 actions have a goal and a result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // To handle feedback from action server
  std::shared_ptr<const typename ActionT::Feedback> feedback_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while waiting for response from a server when a
  // new action goal is sent or canceled
  std::chrono::milliseconds server_timeout_;

  // The timeout value for BT loop execution
  std::chrono::milliseconds bt_loop_duration_;

  // 跟踪发送新目标时动作服务器的确认情况。To track the action server acknowledgement when a new goal is sent
  std::shared_ptr<std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>> //std::shared_future 对象包装了一个异步操作的结果，这个结果是一个指向 ROS 2 动作目标句柄的智能指针
  future_goal_handle_;
  rclcpp::Time time_goal_sent_;
  
  // Can be set in on_tick or on_wait_for_result to indicate if a goal should be sent.
  // 可在 on_tick 或 on_wait_for_result 中设置，以指示是否应发送目标。
  bool should_send_goal_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_

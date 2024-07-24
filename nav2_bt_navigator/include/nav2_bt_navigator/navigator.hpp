// Copyright (c) 2021 Samsung Research America
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

#ifndef NAV2_BT_NAVIGATOR__NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"

namespace nav2_bt_navigator
{

/**
 * @struct FeedbackUtils
 * @brief Navigator feedback utilities required to get transforms and reference frames.
 */

/**
 * @struct FeedbackUtils
 * @brief 导航器反馈工具，用于获取transforms和reference frames 。
 */
struct FeedbackUtils
{
  std::string robot_frame;
  std::string global_frame;
  double transform_tolerance;
  std::shared_ptr<tf2_ros::Buffer> tf;
};

/**
 * @class NavigatorMuxer
 * @brief A class to control the state of the BT navigator by allowing only a single
 * plugin to be processed at a time.
 */

/**
 * @class NavigatorMuxer
 * @brief 一个用于控制BT（行为树）导航器状态的类，通过只允许一次处理单个插件来实现。
 */
class NavigatorMuxer
{
public:
  /**
   * @brief A Navigator Muxer constructor
   */
  NavigatorMuxer()
  : current_navigator_(std::string("")) {}

  /**
   * @brief Get the navigator muxer state
   * @return bool If a navigator is in progress
   */
  bool isNavigating()
  {
    // 使用std::scoped_lock自动管理互斥锁的锁定和解锁
    std::scoped_lock l(mutex_);
    // 检查current_navigator_字符串是否为空，如果不为空表示有导航任务正在进行中
    return !current_navigator_.empty();
  }

/**
 * @brief 用于开始使用给定的导航器名称进行导航。Start navigating with a given navigator
 * @param string Name of the navigator to start
 */
void startNavigating(const std::string & navigator_name) 
{
    std::scoped_lock l(mutex_); // 使用 std::scoped_lock 创建一个作用域锁。当代码块执行完毕后，锁会自动释放。这是为了确保当前操作是线程安全的。
    
    if (!current_navigator_.empty()) { // 检查是否已有导航任务在进行中。如果 current_navigator_ 字符串不为空，说明已经有一个导航任务了。
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"), // 获取名为"NavigatorMutex"的日志记录器
        "Major error! Navigation requested while another navigation"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a navigator plugin."); // 记录一个错误日志，说明请求了一个新的导航任务而当前已经有一个在进行中。这可能是由于导航器插件实现不正确。
    }
    current_navigator_ = navigator_name; // 设置当前的导航器名称为传入的参数 navigator_name，表示开始新的导航任务。
}

  /**
   * @brief Stop navigating with a given navigator
   * @param string Name of the navigator ending task
   */
  void stopNavigating(const std::string & navigator_name)
  {
    std::scoped_lock l(mutex_);
    if (current_navigator_ != navigator_name) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "Major error! Navigation stopped while another navigation"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a navigator plugin."); //"重大错误！导航停止，同时另一个导航""任务正在进行中！这可能是由于""导航插件的执行不正确。
    } else {
      current_navigator_ = std::string("");
    }
  }

protected:
  std::string current_navigator_;
  std::mutex mutex_;
};

/**
 * @class Navigator
 * @brief Navigator interface that acts as a base class for all BT-based Navigator action's plugins
 */

/**
 * @class Navigator
 * @brief Navigator 接口，作为所有基于 BT 的导航action插件的基类
 */
template<class ActionT>
class Navigator
{
public:
  using Ptr = std::shared_ptr<nav2_bt_navigator::Navigator<ActionT>>;

  /**
   * @brief A Navigator constructor
   */
  Navigator()
  {
    plugin_muxer_ = nullptr;
  }

  /**
   * @brief Virtual destructor
   */
  virtual ~Navigator() = default;

  /**
   * @brief Configuration to setup the navigator's backend BT and actions
   * @param parent_node The ROS parent node to utilize
   * @param plugin_lib_names a vector of plugin shared libraries to load
   * @param feedback_utils Some utilities useful for navigators to have
   * @param plugin_muxer The muxing object to ensure only one navigator
   * can be active at a time
   * @param odom_smoother Object to get current smoothed robot's speed
   * @return bool If successful
   */

  /**
 * @brief 配置以设置导航器的后端 BT 和actions
 * @param parent_node 用于使用的 ROS 父节点
 * @param plugin_lib_names 要加载的插件共享库的vector
 * @param feedback_utils 对导航器有用的一些实用工具
 * @param plugin_muxer 一个muxing对象，确保一次只能有一个导航器处于活动状态
 * @param odom_smoother 用于获取当前平滑机器人速度的对象
 * @return bool 如果成功
 */
bool on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node, // 父节点的弱指针
  const std::vector<std::string> & plugin_lib_names, // 插件库的名称列表
  const FeedbackUtils & feedback_utils, // 反馈工具，可能包含与反馈相关的各种工具和设置
  nav2_bt_navigator::NavigatorMuxer * plugin_muxer, // 导航器多路复用器的指针
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) // 里程计平滑处理工具的共享指针
{
  auto node = parent_node.lock(); // 锁定父节点的弱指针，获取强指针
  logger_ = node->get_logger(); // 获取日志记录器
  clock_ = node->get_clock(); // 获取时钟
  feedback_utils_ = feedback_utils; // 设置反馈工具
  plugin_muxer_ = plugin_muxer; // 设置插件多路复用器

  // 获取此导航器的默认行为树文件路径
  std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);

  // 为此导航器创建行为树动作服务器
  bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
    node, // 节点
    getName(), // 获取导航器名称
    plugin_lib_names, // 插件库名称列表
    default_bt_xml_filename, // 默认的行为树XML文件路径
    std::bind(&Navigator::onGoalReceived, this, std::placeholders::_1), // 绑定目标接收回调函数
    std::bind(&Navigator::onLoop, this), // 绑定循环回调函数
    std::bind(&Navigator::onPreempt, this, std::placeholders::_1), // 绑定抢占回调函数
    std::bind(&Navigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2)); // 绑定完成回调函数

  bool ok = true;
  if (!bt_action_server_->on_configure()) { // 调用行为树动作服务器的配置函数
    ok = false; // 如果配置失败，则设置ok为false
  }

  BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard(); // 获取行为树的黑板
  blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf); // 设置tf缓冲区
  blackboard->set<bool>("initial_pose_received", false); // 设置初始姿态接收标志
  blackboard->set<int>("number_recoveries", 0); // 设置恢复次数
  blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother", odom_smoother); // 设置里程计平滑处理工具

  return configure(parent_node, odom_smoother) && ok; // 返回配置结果
}

  /**
   * @brief Activation of the navigator's backend BT and actions
   * @return bool If successful
   */
  bool on_activate()
  {
    bool ok = true;

    if (!bt_action_server_->on_activate()) {
      ok = false;
    }

    return activate() && ok;
  }

  /**
   * @brief Deactivation of the navigator's backend BT and actions
   * @return bool If successful
   */
  bool on_deactivate()
  {
    bool ok = true;
    if (!bt_action_server_->on_deactivate()) {
      ok = false;
    }

    return deactivate() && ok;
  }

  /**
   * @brief Cleanup a navigator
   * @return bool If successful
   */
  bool on_cleanup()
  {
    bool ok = true;
    if (!bt_action_server_->on_cleanup()) {
      ok = false;
    }

    bt_action_server_.reset();

    return cleanup() && ok;
  }

  /**
   * @brief Get the action name of this navigator to expose
   * @return string Name of action to expose
   */

  /**
 * @brief 获取此导航器要公开的动作名称
 * @return string 要公开的动作名称
 */
  virtual std::string getName() = 0;

  virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;

  /**
   * @brief Get the action server
   * @return Action server pointer
   */
  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> & getActionServer()
  {
    return bt_action_server_;
  }

protected:
  /**
   * @brief An intermediate goal reception function to mux navigators.
   */

  /**
 * @brief 一个中间目标接收函数，用于mux导航器。
 */
  bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
  {
    if (plugin_muxer_->isNavigating()) {
      RCLCPP_ERROR(
        logger_,
        "Requested navigation from %s while another navigator is processing,"
        " rejecting request.", getName().c_str()); //从 %s 请求导航，而另一个导航仪正在处理，""拒绝请求。", getName().c_str())；
      return false;
    }

    bool goal_accepted = goalReceived(goal);

    if (goal_accepted) {
      plugin_muxer_->startNavigating(getName());
    }

    return goal_accepted;
  }

  /**
   * @brief 中间完成函数。An intermediate completion function to mux navigators
   */
  void onCompletion(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status)
  {
    plugin_muxer_->stopNavigating(getName());
    goalCompleted(result, final_bt_status);
  }

  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   */

  /**
 * @brief 当 BT 动作服务器接收到一个新目标时要调用的回调函数
 * 可用于检查目标是否有效，并将依赖于接收到的目标的值放在blackboard上
 */
  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */

  /**
 * @brief 一个回调，定义了在 BT 迭代中发生的一次执行
 * 可用于发布动作反馈
 */
  virtual void onLoop() = 0;

  /**
   * @brief A callback that is called when a preempt is requested
   */

  /**
 * @brief 当请求抢占时调用的回调
 */
  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that is called when a the action is completed; Can fill in
   * action result message or indicate that this action is done.
   */

  /**
 * @brief 当动作完成时调用的回调；可以填充动作结果消息或表明此动作已完成。
 */
  virtual void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) = 0;

  /**
   * @brief Method to configure resources.
   */

  /**
 * @brief Method 配置资源。
 */
  virtual bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr /*node*/,
    std::shared_ptr<nav2_util::OdomSmoother>/*odom_smoother*/)
  {
    return true;
  }

  /**
   * @brief Method to cleanup resources.
   */
  virtual bool cleanup() {return true;}

  /**
   * @brief 激活执行中的任何线程的方法。Method to activate any threads involved in execution.
   */
  virtual bool activate() {return true;}

  /**
   * @brief Method to deactivate and any threads involved in execution.
   */
  virtual bool deactivate() {return true;}

  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
  rclcpp::Logger logger_{rclcpp::get_logger("Navigator")};
  rclcpp::Clock::SharedPtr clock_;
  FeedbackUtils feedback_utils_;
  NavigatorMuxer * plugin_muxer_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATOR_HPP_

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

#ifndef NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"


namespace nav2_behavior_tree
{

  /**
   * @enum nav2_behavior_tree::BtStatus
   * @brief An enum class representing BT execution status
   */
  enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

  /**
   * @class nav2_behavior_tree::BehaviorTreeEngine
   * @brief A class to create and handle behavior trees
   */
  class BehaviorTreeEngine
  {
  public:
    /**
     * @brief A constructor for nav2_behavior_tree::BehaviorTreeEngine
     * @param plugin_libraries vector of BT plugin library names to load
     */
    explicit BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries);
    virtual ~BehaviorTreeEngine() {}

    /**
     * @brief Function to execute a BT at a specific rate
     * @param tree BT to execute
     * @param onLoop Function to execute on each iteration of BT execution
     * @param cancelRequested Function to check if cancel was requested during BT execution
     * @param loopTimeout Time period for each iteration of BT execution
     * @return nav2_behavior_tree::BtStatus Status of BT execution
     */

    /**
 * @brief 以特定频率执行 BT 的函数
 * @param tree 要执行的 BT
 * @param onLoop 在 BT 执行的每次迭代中执行的函数
 * @param cancelRequested 在 BT 执行期间检查是否请求了取消的函数
 * @param loopTimeout BT 执行的每次迭代的时间周期
 * @return nav2_behavior_tree::BtStatus BT 执行的状态
 */
    BtStatus run(
        BT::Tree * tree,
        std::function<void()> onLoop,
        std::function<bool()> cancelRequested,
        std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

    /**
     * @brief Function to create a BT from a XML string
     * @param xml_string XML string representing BT
     * @param blackboard Blackboard for BT
     * @return BT::Tree Created behavior tree
     */

    /**
 * @brief 从 XML 字符串创建 BT 的函数
 * @param xml_string 表示 BT 的 XML 字符串
 * @param blackboard BT 的黑板
 * @return BT::Tree 创建的行为树
 */
    BT::Tree createTreeFromText(
        const std::string & xml_string,
        BT::Blackboard::Ptr blackboard);

    /**
     * @brief Function to create a BT from an XML file
     * @param file_path Path to BT XML file
     * @param blackboard Blackboard for BT
     * @return BT::Tree Created behavior tree
     */

    /**
     * @brief 从XML文件创建行为树的函数
     * @param file_path 行为树XML文件的路径
     * @param blackboard 行为树的黑板
     * @return BT::Tree 创建的行为树
     */
    BT::Tree createTreeFromFile(
        const std::string & file_path,
        BT::Blackboard::Ptr blackboard);

    /**
     * @brief Function to explicitly reset all BT nodes to initial state
     * @param root_node Pointer to BT root node
     */


      /**
   * @brief Add groot monitor to publish BT status changes
   * @param tree BT to monitor
   * @param publisher_port ZMQ publisher port for the Groot monitor
   * @param server_port ZMQ server port for the Groot monitor
   * @param max_msg_per_second Maximum number of messages that can be sent per second
   */
  void addGrootMonitoring(
    BT::Tree * tree,
    uint16_t publisher_port,
    uint16_t server_port,
    uint16_t max_msg_per_second = 25);

  /**
   * @brief Reset groot monitor
   */
  void resetGrootMonitor();

    /**
 * @brief 将所有 BT 节点显式重置到初始状态的函数
 * @param root_node 指向 BT 根节点的指针
 */
    void haltAllActions(BT::TreeNode * root_node);

  protected:
    // The factory that will be used to dynamically construct the behavior tree
    // 将用于动态构造行为树的工厂
    BT::BehaviorTreeFactory factory_;

    static inline std::unique_ptr<BT::PublisherZMQ> groot_monitor_;
  };

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

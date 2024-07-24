#include <string>

#include "nav2_behavior_tree/plugins/condition/is_aligned_charge_condition.hpp"

namespace nav2_behavior_tree
{
IsAlignedChargeCondition::IsAlignedChargeCondition(const std::string &condition_name,
                                                   const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf),
      dock_alignment_topic_("/dock_alignment_status"),
      is_aligned_charge_(false)
{
    getInput("dock_alignment_topic", dock_alignment_topic_);
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);

    callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    // 创建订阅，订阅电池状态消息
    // 当收到消息时，调用batteryCallback函数
    dock_alignment_sub_ = node->create_subscription<nav2_msgs::msgs::DockAlignmentStatus>(
        dock_alignment_topic_,                                                                    // 订阅的话题
        rclcpp::SystemDefaultsQoS(),                                                              // 使用系统默认的服务质量设置
        std::bind(&IsAlignedChargeCondition::dockAlignmentCallback, this, std::placeholders::_1), // 绑定的回调函数
        sub_option);                                                                              // 订阅选项，包括回调组
}

BT::NodeStatus IsAlignedChargeCondition::tick()
{
    callback_group_executor_.spin_some();
    if (is_aligned_charge_)
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void IsAlignedChargeCondition::dockAlignmentCallback(nav2_msgs::msg::DockAlignmentStatus::SharedPtr msg)
{
    is_aligned_charge_ = msg->is_aligned_charge;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsAlignedChargeCondition>("IsAlignedCharge");
}
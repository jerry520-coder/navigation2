#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ALIGNED_CHARGE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ALIGNED_CHARGE_CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/dock_alignment_status.hpp"

namespace nav2_behavior_tree
{

    /**
     * @brief IsAlignedChargeCondition 默认是false
     * 
     */
class IsAlignedChargeCondition : public BT::ConditionNode
{
public:
    IsAlignedChargeCondition(const std::string &condition_name,
                             const BT::NodeConfiguration &conf);

    IsAlignedChargeCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(
            "dock_alignment_topic", std::string("/dock_alignment_status"), "Dock alignment topic")};
    }


private:
    void dockAlignmentCallback(nav2_msgs::msg::DockAlignmentStatus::SharedPtr msg);

    
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::Subscription<nav2_msgs::msgs::DockAlignmentStatus>::SharedPtr dock_alignment_sub_;
    std::string dock_alignment_topic_;
    bool is_aligned_charge_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_CHARGING_CONDITION_HPP_
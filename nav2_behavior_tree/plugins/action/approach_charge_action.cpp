#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/approach_charge_action.hpp"

namespace nav2_behavior_tree
{
ApproachChargeAction::ApproachChargeAction(
    const std::string &xml_tag_name,
    const std::string &action_name,
    const BT::NodeConfiguration &conf)
    : BtActionNode<nav2_msgs::action::ApproachCharge>(xml_tag_name, action_name, conf)
{
    // getInput("found_charging_dock", found_charging_dock_);
}

void ApproachChargeAction::on_tick()
{
}

void ApproachChargeAction::on_wait_for_result(
    std::shared_ptr<const nav2_msgs::action::ApproachCharge::Feedback> feedback)
{
}

BT::NodeStatus ApproachChargeAction::on_success()
{
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachChargeAction::on_aborted()
{
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ApproachChargeAction::on_cancelled()
{
    return BT::NodeStatus::SUCCESS;
}

void ApproachChargeAction::halt()
{
    BtActionNode::halt();
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<nav2_behavior_tree::ApproachChargeAction>(
                name, "approach_charge", config);
        };

    factory.registerBuilder<nav2_behavior_tree::ApproachChargeAction>(
        "ApproachCharge", builder);
}
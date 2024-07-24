#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__APPROACH_CHARGE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__APPROACH_CHARGE_ACTION_HPP_

#include <memory>
#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/approach_charge.hpp"

namespace nav2_behavior_tree
{
class ApproachChargeAction : public BtActionNode<nav2_msgs::action::ApproachCharge>
{
public:
    ApproachChargeAction(const std::string &xml_tag_name,
                         const std::string &action_name,
                         const BT::NodeConfiguration &conf);

    void on_tick() override;

    BT::NodeStatus on_success() override;

    BT::NodeStatus on_aborted() override;

    BT::NodeStatus on_cancelled() override;

    void halt() override;

    void on_wait_for_result(
        std::shared_ptr<const nav2_msgs::action::ApproachCharge::Feedback> feedback) override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
            {
                BT::InputPort<bool>("found_charging_dock", "Whether charging pile is detected."),
            });
    }

private:
    bool found_charging_dock_;
};

} // namespace nav2_behavior_tree

#endif //NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__APPROACH_CHARGE_ACTION_HPP_
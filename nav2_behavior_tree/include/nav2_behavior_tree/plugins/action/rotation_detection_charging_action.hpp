#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ROTATION_DETECTION_CHARGING_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ROTATION_DETECTION_CHARGING_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "your_package_name/action/charge_dock_detection.hpp"
#include "actionlib/client/simple_action_client.h"

namespace nav2_behavior_tree
{

class RotationDetectionChargingAction : public BtActionNode<nav2_msgs::action::Spin>
{
public:
  RotationDetectionChargingAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts();

private:
  void dockDetectionResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const your_package_name::action::ChargeDockDetection::ResultConstPtr& result);

  actionlib::SimpleActionClient<your_package_name::action::ChargeDockDetection> dock_detection_action_client_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ROTATION_DETECTION_CHARGING_ACTION_HPP_

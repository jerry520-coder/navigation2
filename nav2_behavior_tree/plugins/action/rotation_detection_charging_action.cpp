#include "rotation_detection_charging_action.hpp"

namespace nav2_behavior_tree
{

RotationDetectionChargingAction::RotationDetectionChargingAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
  : BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf),
    dock_detection_action_client_("charge_dock_detection", true)
{
  // 构造函数初始化
}

void RotationDetectionChargingAction::on_tick()
{
  BtActionNode<nav2_msgs::action::Spin>::on_tick();
  // 初始化充电桩检测
  auto goal = your_package_name::action::ChargeDockDetection::Goal();
  // 根据需要自定义目标
  dock_detection_action_client_.sendGoal(goal,
    std::bind(&RotationDetectionChargingAction::dockDetectionResultCallback, this, std::placeholders::_1));
}

BT::PortsList RotationDetectionChargingAction::providedPorts()
{
  return providedBasicPorts(
    {
      BT::InputPort<double>("spin_dist", 1.57, "Spin distance"),
      BT::InputPort<double>("time_allowance", 10.0, "Allowed time for spinning"),
      // 可以在这里添加针对充电桩检测的额外端口
    });
}

void RotationDetectionChargingAction::dockDetectionResultCallback(
  const actionlib::SimpleClientGoalState& state,
  const your_package_name::action::ChargeDockDetection::ResultConstPtr& result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED && result->found_dock)
  {
    RCLCPP_INFO(this->node_->get_logger(), "Charging dock detected.");
    // 处理充电桩检测成功
  }
  else
  {
    RCLCPP_INFO(this->node_->get_logger(), "Charging dock not detected.");
    // 处理失败或继续旋转
  }
}

}  // namespace nav2_behavior_tree

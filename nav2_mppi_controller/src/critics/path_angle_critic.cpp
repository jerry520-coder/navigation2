// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#include "nav2_mppi_controller/critics/path_angle_critic.hpp"

#include <math.h>

namespace mppi::critics
{

void PathAngleCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  float vx_min;
  getParentParam(vx_min, "vx_min", -0.35);
  if (fabs(vx_min) < 1e-6) {  // zero
    reversing_allowed_ = false;
  } else if (vx_min < 0.0) {   // reversing possible
    reversing_allowed_ = true;
  }

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(offset_from_furthest_, "offset_from_furthest", 4);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 2.0);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5);
  getParam(
    max_angle_to_furthest_,
    "max_angle_to_furthest", 1.2);
  getParam(
    forward_preference_,
    "forward_preference", true);

  if (!reversing_allowed_) {
    forward_preference_ = true;
  }

  RCLCPP_INFO(
    logger_,
    "PathAngleCritic instantiated with %d power and %f weight. Reversing %s",
    power_, weight_, reversing_allowed_ ? "allowed." : "not allowed.");
}

void PathAngleCritic::score(CriticData & data) 
{
  // 使用xt库的即时评估策略
  using xt::evaluation_strategy::immediate;
  // 如果该批评者未启用，则直接返回
  if (!enabled_) {
    return;
  }

  // 如果机器人已经在目标位置容忍范围内，则不计算成本
  if (utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path)) {
    return;
  }

  // 如果未设置，则设置路径最远点
  utils::setPathFurthestPointIfNotSet(data);

  // 计算偏移后的路径点索引，确保不会超出路径点索引范围
  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, data.path.x.shape(0) - 1);

  // 获取目标点的x和y坐标
  const float goal_x = xt::view(data.path.x, offseted_idx);
  const float goal_y = xt::view(data.path.y, offseted_idx);

  // 如果从当前姿态到目标点的角度小于最大允许角度，则不计算成本
  if (utils::posePointAngle(
      data.state.pose.pose, goal_x, goal_y, forward_preference_) < max_angle_to_furthest_)
  {
    return;
  }

  // 计算两个点之间的偏航角
  auto yaws_between_points = xt::atan2(
    goal_y - data.trajectories.y,
    goal_x - data.trajectories.x);

  // 计算实际偏航角与目标偏航角之间的最短角度差
  auto yaws =
    xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points));

  // 如果允许倒车并且机器人没有前进方向的偏好
  if (reversing_allowed_ && !forward_preference_) {
    // 纠正偏航角，使其在允许的范围内
    const auto yaws_between_points_corrected = xt::where(
      yaws < M_PI_2, yaws_between_points, utils::normalize_angles(yaws_between_points + M_PI));
    // 计算纠正后的偏航角差
    const auto corrected_yaws = xt::abs(
      utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points_corrected));
    // 累加成本，考虑权重和指数
    data.costs += xt::pow(xt::mean(corrected_yaws, {1}, immediate) * weight_, power_);
  } else {
    // 如果机器人有前进方向的偏好或者不允许倒车，则直接累加成本
    data.costs += xt::pow(xt::mean(yaws, {1}, immediate) * weight_, power_);
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAngleCritic,
  mppi::critics::CriticFunction)

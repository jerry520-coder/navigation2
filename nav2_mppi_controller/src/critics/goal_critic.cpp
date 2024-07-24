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

#include "nav2_mppi_controller/critics/goal_critic.hpp"

namespace mppi::critics
{

using xt::evaluation_strategy::immediate;

void GoalCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0);
  getParam(threshold_to_consider_, "threshold_to_consider", 1.4);

  RCLCPP_INFO(
    logger_, "GoalCritic instantiated with %d power and %f weight.",
    power_, weight_);
}

void GoalCritic::score(CriticData & data)
{
  // 如果未启用或者不在目标位置阈值内，则直接返回
  if (!enabled_ || !utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  // 获取路径的最后一个索引和目标位置
  const auto goal_idx = data.path.x.shape(0) - 1;
  const auto goal_x = data.path.x(goal_idx);
  const auto goal_y = data.path.y(goal_idx);

  // 从轨迹数据中提取 x 和 y 坐标
  const auto traj_x = xt::view(data.trajectories.x, xt::all(), xt::all());
  const auto traj_y = xt::view(data.trajectories.y, xt::all(), xt::all());

  // 计算轨迹点到目标点的距离
  auto dists = xt::sqrt(
    xt::pow(traj_x - goal_x, 2) +
    xt::pow(traj_y - goal_y, 2));

  // 计算平均距离并将其加入总成本
  // 公式：costs += $pow(mean(dists, {1}, immediate) * weight, power)$
  data.costs += xt::pow(xt::mean(dists, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalCritic, mppi::critics::CriticFunction)

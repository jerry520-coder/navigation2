// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/critics/goal_angle_critic.hpp"

namespace mppi::critics
{

void GoalAngleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.0);

  getParam(threshold_to_consider_, "threshold_to_consider", 0.5);

  RCLCPP_INFO(
    logger_,
    "GoalAngleCritic instantiated with %d power, %f weight, and %f "
    "angular threshold.",
    power_, weight_, threshold_to_consider_);
}

void GoalAngleCritic::score(CriticData & data)
{
  // 如果未启用或者不在目标位置阈值内，则直接返回
  if (!enabled_ || !utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  //获取路径的最后一个索引和目标航向角
  //path为截取的局部路径 
  const auto goal_idx = data.path.x.shape(0) - 1;  //使用 x.shape(0) 获取路径的长度，减去 1，得到最后一个点的索引 goal_idx。
  const float goal_yaw = data.path.yaws(goal_idx);

  // 计算角度差值并添加到总成本中
  data.costs += xt::pow(
    xt::mean(xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw)), {1}) *
    weight_, power_);

  //xt::mean 计算每一行的均值
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GoalAngleCritic,
  mppi::critics::CriticFunction)

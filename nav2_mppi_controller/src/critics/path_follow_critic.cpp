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

#include "nav2_mppi_controller/critics/path_follow_critic.hpp"

#include <xtensor/xmath.hpp>
#include <xtensor/xsort.hpp>

namespace mppi::critics
{

void PathFollowCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 1.4);
  getParam(offset_from_furthest_, "offset_from_furthest", 6);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0);
}

void PathFollowCritic::score(CriticData & data) 
{
  // 如果该批评者未启用，路径点少于2个，或者机器人已经在目标位置容忍范围内，则不计算成本
  if (!enabled_ || data.path.x.shape(0) < 2 || 
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path)) 
  {
    return;
  }

  // 如果未设置，则设置路径最远点
  utils::setPathFurthestPointIfNotSet(data);
  // 如果未设置，则设置路径成本
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  // 获取路径点的数量
  const size_t path_size = data.path.x.shape(0) - 1;

  // 计算偏移后的路径点索引，确保不会超出路径点索引范围
  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, path_size);

  // 寻找第一个有效的路径点，以应对路径上的动态障碍物
  // 我们希望越过障碍物，而不是穿过它
  bool valid = false;
  while (!valid && offseted_idx < path_size - 1) {
    valid = (*data.path_pts_valid)[offseted_idx];
    if (!valid) {
      offseted_idx++;
    }
  }

  // 获取当前评估的路径点的x和y坐标
  const auto path_x = data.path.x(offseted_idx);
  const auto path_y = data.path.y(offseted_idx);

  // 获取所有轨迹的最最后一个点的x和y坐标
  const auto last_x = xt::view(data.trajectories.x, xt::all(), -1);
  const auto last_y = xt::view(data.trajectories.y, xt::all(), -1);

  // 计算每个轨迹最后一个点到当前评估路径点的距离
  auto dists = xt::sqrt(
    xt::pow(last_x - path_x, 2) +
    xt::pow(last_y - path_y, 2));

  // 将计算出的距离乘以权重并按指数次幂计算，然后累加到总成本中
  data.costs += xt::pow(weight_ * std::move(dists), power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathFollowCritic,
  mppi::critics::CriticFunction)

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

#include "nav2_mppi_controller/critics/prefer_forward_critic.hpp"

namespace mppi::critics
{

void PreferForwardCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5);

  RCLCPP_INFO(
    logger_, "PreferForwardCritic instantiated with %d power and %f weight.", power_, weight_);
}

void PreferForwardCritic::score(CriticData & data) 
{
  // 使用xt库的即时评估策略
  using xt::evaluation_strategy::immediate;
  // 如果该批评者未启用，或者机器人已经在目标位置容忍范围内，则不计算成本
  if (!enabled_ ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path)) {
    return;
  }

  // 计算机器人的后退运动速度，如果速度为负，则取其绝对值
  // 这里使用xt::maximum函数来确保速度不会小于0，即只考虑后退时的速度
  auto backward_motion = xt::maximum(-data.state.vx, 0);

  // 累加成本，考虑后退运动的速度、模型的时间步长（data.model_dt）和权重
  // xt::sum函数对所有后退速度值求和，然后乘以时间步长和权重
  // 之后使用xt::pow函数将求和结果按指定的指数次幂计算成本
  data.costs += xt::pow(
    xt::sum(
      std::move(
        backward_motion) * data.model_dt, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PreferForwardCritic,
  mppi::critics::CriticFunction)

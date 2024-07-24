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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_

#include <memory>
#include <vector>
#include <xtensor/xtensor.hpp>
// #include <optional> 


#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/motion_models.hpp"


namespace mppi
{

/**
 * @struct mppi::CriticData
 * @brief 状态state / 轨迹trajectories / 路径path / 成本costs / 模型时间间隔model_dt / 失败标志fail_flag / 目标检查器goal_checker / 
 * 运动模型motion_model / 可选的 路径点有效性path_pts_valid / 可选的 最远到达的路径点furthest_reached_path_point
 * Data to pass to critics for scoring, including state, trajectories, path, costs, and
 * important parameters to share
 * @note   CriticData critics_data_ = {state_, generated_trajectories_, path_, costs_, settings_.model_dt, false, nullptr, nullptr,std::nullopt, std::nullopt};
 */
struct CriticData
{
  // 状态
  const models::State & state;

  // 轨迹
  const models::Trajectories & trajectories;

  // 路径
  const models::Path & path;

  // 成本
  xt::xtensor<float, 1> & costs;

  // 模型时间间隔
  float & model_dt;

  // 失败标志
  bool fail_flag;

  // 目标检查器
  nav2_core::GoalChecker * goal_checker;

  // 运动模型
  std::shared_ptr<MotionModel> motion_model;

  // 可选的 路径点有效性
  std::optional<std::vector<bool>> path_pts_valid; //当需要表示一个值可能存在，但有时也可能不存在的情况时，可以使用 std::optional。这样可以更清晰地表达出程序的意图，并且在使用这些变量时，需要通过 std::optional 的方法来检查是否有值，并且可以安全地访问这些值。

  // 可选的 最远到达的路径点
  std::optional<size_t> furthest_reached_path_point;
};


}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_

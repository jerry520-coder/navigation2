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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_

#include <xtensor/xtensor.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::State
 * @brief State information: velocities, controls, poses, speed
 */
struct State
{
  // 表示 x 方向上的速度
  xt::xtensor<float, 2> vx;

  // 表示 y 方向上的速度
  xt::xtensor<float, 2> vy;

  // 表示 z 轴上的角速度
  xt::xtensor<float, 2> wz;

  // 表示 x 方向上的控制速度
  xt::xtensor<float, 2> cvx;

  // 表示 y 方向上的控制速度
  xt::xtensor<float, 2> cvy;

  // 表示 z 轴上的控制角速度
  xt::xtensor<float, 2> cwz;

  // 表示当前的姿态信息
  geometry_msgs::msg::PoseStamped pose;

  // 表示当前的速度信息
  geometry_msgs::msg::Twist speed;

  /**
    * @brief 重置状态数据
    * @param batch_size 批处理大小
    * @param time_steps 时间步长
    */
  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    // 将各个成员变量的值初始化为零矩阵，大小为 batch_size x time_steps
    vx = xt::zeros<float>({batch_size, time_steps});
    vy = xt::zeros<float>({batch_size, time_steps});
    wz = xt::zeros<float>({batch_size, time_steps});

    cvx = xt::zeros<float>({batch_size, time_steps});
    cvy = xt::zeros<float>({batch_size, time_steps});
    cwz = xt::zeros<float>({batch_size, time_steps});
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_

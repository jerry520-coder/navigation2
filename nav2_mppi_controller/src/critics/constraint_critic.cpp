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

#include "nav2_mppi_controller/critics/constraint_critic.hpp"

namespace mppi::critics
{

void ConstraintCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0);
  RCLCPP_INFO(
    logger_, "ConstraintCritic instantiated with %d power and %f weight.",
    power_, weight_);

  float vx_max, vy_max, vx_min;
  getParentParam(vx_max, "vx_max", 0.5);
  getParentParam(vy_max, "vy_max", 0.0);
  getParentParam(vx_min, "vx_min", -0.35);

  const float min_sgn = vx_min > 0.0 ? 1.0 : -1.0;
  max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);
}

void ConstraintCritic::score(CriticData & data)
{
  // 使用 immediate 策略来评估表达式
  using xt::evaluation_strategy::immediate;  //用于一些可以延迟或立即计算结果的函数，比如 xt::sum ()，xt::mean () 等。这意味着函数会立即计算结果，并将其存储在一个新的数组中。这样做的好处是可以避免重复计算，但是缺点是会增加内存开销。

  // 如果批评家未启用，则直接返回
  if (!enabled_) {
    return;
  }

  // 根据车辆速度的正负来设置 sgn 的值，用于后续计算
  auto sgn = xt::where(data.state.vx > 0.0, 1.0, -1.0);

  // 计算车辆的总速度
  auto vel_total = sgn * xt::sqrt(data.state.vx * data.state.vx + data.state.vy * data.state.vy);

  // 计算超出最大速度限制的运动
  auto out_of_max_bounds_motion = xt::maximum(vel_total - max_vel_, 0);

  // 计算低于最小速度限制的运动
  auto out_of_min_bounds_motion = xt::maximum(min_vel_ - vel_total, 0);

  // 如果 motion_model 是 AckermannMotionModel 类型的指针，则进行下面的操作
  auto acker = dynamic_cast<AckermannMotionModel *>(data.motion_model.get()); //用于将基类指针或引用转换为派生类指针或引用。这种转换是安全的，会检查转换是否合法，如果不合法则返回 nullptr（对指针进行转换）或抛出异常（对引用进行转换）。
  if (acker != nullptr) {
    auto & vx = data.state.vx;
    auto & wz = data.state.wz;

    // 计算超出最大转弯半径限制的运动
    auto out_of_turning_rad_motion = xt::maximum(
      acker->getMinTurningRadius() - (xt::fabs(vx) / xt::fabs(wz)), 0.0);

    // 计算总的代价，并加到 data.costs 中
    data.costs += xt::pow(
      xt::sum(
        (std::move(out_of_max_bounds_motion) +
        std::move(out_of_min_bounds_motion) +
        std::move(out_of_turning_rad_motion)) *
        data.model_dt, {1}, immediate) * weight_, power_);

// std::move 用于将对象的所有权转移，以便更高效地传递资源所有权或避免不必要的拷贝。在这里，out_of_max_bounds_motion 的内容被移动到 xt::sum 函数中进行求和操作，表示不再需要 out_of_max_bounds_motion 的内容。
// {1}：对列求和
// immediate：这是之前提到的评估策略，表示立即计算这个表达式的结果。
// weight_：这是一个常量或变量，代表权重。
// power_：这是一个常量或变量，代表幂次。

    // 返回，结束函数
    return;
  }

  // 如果 motion_model 不是 AckermannMotionModel 类型的指针，则进行下面的操作

  // 计算总的代价，并加到 data.costs 中
  data.costs += xt::pow(
    xt::sum(
      (std::move(out_of_max_bounds_motion) +
      std::move(out_of_min_bounds_motion)) *
      data.model_dt, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ConstraintCritic, mppi::critics::CriticFunction)

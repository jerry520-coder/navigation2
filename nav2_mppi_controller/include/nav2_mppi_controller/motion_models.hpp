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

#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <cstdint>

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel
{
public:
  /**
    * @brief Constructor for mppi::MotionModel
    */
  MotionModel() = default;

  /**
    * @brief Destructor for mppi::MotionModel
    */
  virtual ~MotionModel() = default;

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */

  /**
 * @brief 根据输入速度，预测车辆的输出速度
 * @param state 包含控制速度的状态对象，用于生成车辆的输出速度
 */
virtual void predict(models::State & state)
{
  using namespace xt::placeholders;  // NOLINT

  // 使用输入速度来预测输出速度
  // state.vx 是车辆的 x 方向速度
  // state.cvx 是输入的控制速度
  // 从第二列开始，将输入的控制速度赋值给车辆的 x 方向速度
  xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
    xt::view(state.cvx, xt::all(), xt::range(0, -1)); //将 state.cvx 的第一列（即 xt::range(0, -1)，表示从第 0 列到倒数第二列）赋值给 state.vx 的第二列（即 xt::range(1, _)，表示从第 1 列到最后一列）

  // 类似地，将输入的控制角速度赋值给车辆的 z 方向速度
  xt::noalias(xt::view(state.wz, xt::all(), xt::range(1, _))) =
    xt::view(state.cwz, xt::all(), xt::range(0, -1));

  // 如果是全向机器人，将输入的控制速度赋值给车辆的 y 方向速度
  if (isHolonomic()) {
    xt::noalias(xt::view(state.vy, xt::all(), xt::range(1, _))) =
      xt::view(state.cvy, xt::all(), xt::range(0, -1));
  }
}

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief 对控制序列应用硬车约束。Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann motion model
 */
class AckermannMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::AckermannMotionModel
    */
  explicit AckermannMotionModel(ParametersHandler * param_handler)
  {
    auto getParam = param_handler->getParamGetter("AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & vx = control_sequence.vx;  // 获取控制序列的 x 方向速度
    auto & wz = control_sequence.wz;  // 获取控制序列的 z 方向速度

    // 使用 masked_view 创建掩码视图，将满足约束条件的部分取出
    // fabs(vx) / fabs(wz) < min_turning_r_ 是一个布尔掩码
    auto view = xt::masked_view(wz, xt::fabs(vx) / xt::fabs(wz) < min_turning_r_); //根据转弯半径的约束条件从控制序列的 wz 数组中选择满足条件的部分元素，这些被选择的元素会在接下来的操作中被修改为符合约束条件的新值

    // 将满足约束条件的部分用新的值替换
    // 新的值为 sign(wz) * vx / min_turning_r_
    // 即符号相同，按比例缩小
    view = xt::sign(wz) * vx / min_turning_r_;
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() {return min_turning_r_;}

private:
  float min_turning_r_{0};
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::DiffDriveMotionModel
    */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }
};

/**
 * @class mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 */
class OmniMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::OmniMotionModel
    */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return true;
  }
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

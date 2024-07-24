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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for aligning to path in cases of extreme misalignment
 * or turning
 */

/**
 * @class mppi::critics::ConstraintCritic
 * @brief 在极端错位或转弯情况下，用于对齐路径的批评者目标函数
 */
class PathAngleCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */

  /**
 * @brief 计算机器人在目标位姿上的朝向相关成本
 * （仅在机器人接近current plan中的最后一个目标时考虑）
 *
 * @param costs [out] 将目标角度成本值添加到此张量中
 */
  void score(CriticData & data) override;

protected:
  float max_angle_to_furthest_{0};
  float threshold_to_consider_{0};

  size_t offset_from_furthest_{0};
  bool reversing_allowed_{true};
  bool forward_preference_{true};

  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_

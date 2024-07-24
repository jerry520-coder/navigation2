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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_

#include <cstddef>
#include "nav2_mppi_controller/models/constraints.hpp"

namespace mppi::models
{

/**
 * @struct mppi::models::OptimizerSettings
 * @brief Settings for the optimizer to use
 */
struct OptimizerSettings
{
  models::ControlConstraints base_constraints{0, 0, 0, 0};  /**< 基础控制约束 */
  models::ControlConstraints constraints{0, 0, 0, 0};      /**< 控制约束 */
  models::SamplingStd sampling_std{0, 0, 0};               /**< 采样标准差 */
  float model_dt{0};                                       /**< 模型时间步长 */
  float temperature{0};                                    /**< 温度参数 */
  float gamma{0};                                          /**< Gamma 参数 */
  unsigned int batch_size{0};                              /**< 批量大小 */
  unsigned int time_steps{0};                              /**< 时间步数 */
  unsigned int iteration_count{0};                         /**< 迭代次数 */
  bool shift_control_sequence{false};                      /**< 是否移动控制序列 */
  size_t retry_attempt_limit{0};                           /**< 重试尝试限制 */
};


}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_

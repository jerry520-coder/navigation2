/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Samsung Research Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the <ORGANIZATION> nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Alexey Merzlyakov
 *********************************************************************/

#ifndef NAV2_COSTMAP_2D__COSTMAP_FILTERS__FILTER_VALUES_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_FILTERS__FILTER_VALUES_HPP_

/** Provides constants used in costmap filters */

namespace nav2_costmap_2d
{

/** 成本图过滤器类型 */
static constexpr uint8_t KEEPOUT_FILTER = 0; // 避障过滤器，用于标识应避免的区域
static constexpr uint8_t SPEED_FILTER_PERCENT = 1; // 速度百分比过滤器，用于根据地图区域调整速度的百分比
static constexpr uint8_t SPEED_FILTER_ABSOLUTE = 2; // 绝对速度过滤器，用于指定特定区域的具体速度限制
static constexpr uint8_t BINARY_FILTER = 3; // 二进制过滤器，通常用于有无障碍的简单判断

/** 基础值和乘数的默认值 */
static constexpr double BASE_DEFAULT = 0.0; // 基础值默认为0.0，用于初始化或默认配置
static constexpr double MULTIPLIER_DEFAULT = 1.0; // 乘数默认为1.0，表示默认不改变原有值

/** 速度过滤器常量 */
static constexpr int8_t SPEED_MASK_UNKNOWN = -1; // 表示速度限制未知
static constexpr int8_t SPEED_MASK_NO_LIMIT = 0; // 表示无速度限制
static constexpr double NO_SPEED_LIMIT = 0.0; // 用0.0表示没有速度限制


}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_FILTERS__FILTER_VALUES_HPP_

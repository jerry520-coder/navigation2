/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__COST_VALUES_HPP_
#define NAV2_COSTMAP_2D__COST_VALUES_HPP_
/** Provides a mapping for often used cost values */
namespace nav2_costmap_2d
{
// 通常是为了在编译时计算出一个常量，并且该常量在整个程序的生命周期内保持其值。constexpr 用于声明编译时常量，而 static 用于指定静态存储持续时间。
static constexpr unsigned char NO_INFORMATION = 255;//代表无信息区域
static constexpr unsigned char LETHAL_OBSTACLE = 254;//代表致命障碍
static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;//代表膨胀障碍的内环。这通常用于机器人的路径规划，表示机器人的边缘如果触碰到该区域，可能会导致碰撞。实际上，这是一个安全缓冲区域，让机器人在保持距离的同时避开障碍。
static constexpr unsigned char MAX_NON_OBSTACLE = 252;//最大非障碍值，用于表示此值以下（不包括该值）的所有值都是非障碍区域。这可能包括一些安全通过的低风险区域。
static constexpr unsigned char FREE_SPACE = 0; //自由空间，即没有任何障碍的区域，机器人可以安全通过。
}
#endif  // NAV2_COSTMAP_2D__COST_VALUES_HPP_

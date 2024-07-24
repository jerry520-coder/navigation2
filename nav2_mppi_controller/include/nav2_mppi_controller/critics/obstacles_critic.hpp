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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_

#include <memory>
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for avoiding obstacles, allowing it to deviate off
 * the planned path. This is important to tune in tandem with PathAlign to make a balance
 * between path-tracking and dynamic obstacle avoidance capabilities as desirable for a
 * particular application
 */

/**
 * @class mppi::critics::ConstraintCritic
 * @brief 用于避开障碍物的评价器目标函数，允许偏离规划路径。
 * 这对于与 PathAlign 调节以达到一种在特定应用中平衡路径跟踪和动态障碍物避免能力非常重要。
 */
class ObstaclesCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  /**
    * @brief Checks if cost represents a collision
    * @param cost Costmap cost
    * @return bool if in collision
    */
  inline bool inCollision(float cost) const;

  /**
    * @brief cost at a robot pose
    * @param x X of pose
    * @param y Y of pose
    * @param theta theta of pose
    * @return Collision information at pose
    */
  inline CollisionCost costAtPose(float x, float y, float theta);

  /**
    * @brief Distance to obstacle from cost
    * @param cost Costmap cost
    * @return float Distance to the obstacle represented by cost
    */
  inline float distanceToObstacle(const CollisionCost & cost);

  /**
    * @brief Find the min cost of the inflation decay function for which the robot MAY be
    * in collision in any orientation
    * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
    * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
    * since some element of the robot could be in collision
    */

  /**
   * @brief 寻找充分膨胀衰减函数的最小成本，其中机器人可能在任何方向上与障碍物发生碰撞
   * @param costmap Costmap2DROS，用于获取最小内切成本（例如膨胀层文档中的 128）
   * @return double 外接圆（circumscribed）成本，高于此值需要进行完整的足迹碰撞检查，因为机器人的某些部分可能发生碰撞
   */
  float findCircumscribedCost(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap);


protected:
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  collision_checker_{nullptr};

  bool consider_footprint_{true};
  float collision_cost_{0};
  float inflation_scale_factor_{0}, inflation_radius_{0};

  float possibly_inscribed_cost_;
  float collision_margin_distance_;
  float near_goal_distance_;
  float circumscribed_cost_{0}, circumscribed_radius_{0};

  unsigned int power_{0};
  float repulsion_weight_, critical_weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_

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

#include <cmath>
#include "nav2_mppi_controller/critics/obstacles_critic.hpp"

namespace mppi::critics
{

void ObstaclesCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 1);
  getParam(repulsion_weight_, "repulsion_weight", 1.5);
  getParam(critical_weight_, "critical_weight", 20.0);
  getParam(collision_cost_, "collision_cost", 10000.0);
  getParam(collision_margin_distance_, "collision_margin_distance", 0.10);
  getParam(near_goal_distance_, "near_goal_distance", 0.5);

  collision_checker_.setCostmap(costmap_);
  possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);

// "膨胀层要么未找到，要么膨胀设置不足以优化非圆形碰撞检查功能。强烈建议将膨胀半径设置为机器人最大横截面的至少一半。
// 请参阅 github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields 以获取完整的说明。这将极大地影响运行时性能。"
  if (possibly_inscribed_cost_ < 1.0f) {
    RCLCPP_ERROR(
      logger_,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  RCLCPP_INFO(
    logger_,
    "ObstaclesCritic instantiated with %d power and %f / %f weights. "
    "Critic will collision check based on %s cost.",
    power_, critical_weight_, repulsion_weight_, consider_footprint_ ?
    "footprint" : "circular");
}

float ObstaclesCritic::findCircumscribedCost(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  double result = -1.0;
  bool inflation_layer_found = false;

  // 获取外接圆半径
  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();

  // 如果外接圆半径与上一次的值相同，则直接返回之前计算的结果
  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
// early return if footprint size is unchanged
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  for (auto layer = costmap->getLayeredCostmap()->getPlugins()->begin();
    layer != costmap->getLayeredCostmap()->getPlugins()->end();
    ++layer)
  {
    auto inflation_layer = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer); //用于在运行时将一个 std::shared_ptr 强制转换为另一种类型的 std::shared_ptr
    if (!inflation_layer) {
      continue;
    }

    //nullptr则执行

    inflation_layer_found = true;
    // 获取 costmap 分辨率
    const double resolution = costmap->getCostmap()->getResolution();
    // 计算外接圆的成本
    result = inflation_layer->computeCost(circum_radius / resolution);
    // 从参数处理程序获取参数
    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(inflation_scale_factor_, "cost_scaling_factor", 10.0);
    getParam(inflation_radius_, "inflation_radius", 0.55);
  }

  // 如果未找到膨胀层，则打印警告信息
  if (!inflation_layer_found) {

  // "未在costmap配置中找到膨胀层。"
  // "如果这是一个SE2-碰撞检测插件，它无法使用costmap潜在场来加速碰撞检测，"
  // "仅在机器人在可能内切半径的障碍物附近时检查完整的足迹。"
  // "这可能会显著减慢规划时间，并且仅避免绝对碰撞之外的任何事物！"
    RCLCPP_WARN(
      logger_,
      "No inflation layer found in costmap configuration. "
      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
      "field to speed up collision checking by only checking the full footprint "
      "when robot is within possibly-inscribed radius of an obstacle. This may "
      "significantly slow down planning times and not avoid anything but absolute collisions!");
  }

  // 更新外接圆半径和成本值
  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);

  return circumscribed_cost_;
}

float ObstaclesCritic::distanceToObstacle(const CollisionCost & cost)
{
  // 膨胀因子
  const float scale_factor = inflation_scale_factor_;
  // 获取最小内接半径
  const float min_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  // 计算到障碍物的距离
  // 这里的计算基于指数式，用于将碰撞成本转换为距离
  // 公式为：(scale_factor * min_radius - log(cost.cost) + log(253.0f)) / scale_factor
  // 其中 cost.cost 是碰撞成本，越大表示越接近障碍物，log 用于转换成对数域
  // 253.0f 是一个常数，用于调整转换的基数
  float dist_to_obj = (scale_factor * min_radius - log(cost.cost) + log(253.0f)) / scale_factor;

  // 如果不是使用足迹的碰撞检查，说明成本是使用中心点的成本
  // 需要减去最小半径，以获得到对象的最近距离
  // If not footprint collision checking, the cost is using the center point cost and
  // needs the radius subtracted to obtain the closest distance to the object
  if (!cost.using_footprint) {
    dist_to_obj -= min_radius;
  }

  return dist_to_obj;
}

void ObstaclesCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_) {
    return;
  }

  // 如果考虑了足迹（footprint），重新计算可能的最大内切成本
  if (consider_footprint_) {
    // 足迹可能在初始化后发生变化，如果用户使用动态足迹
    possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
  }

  // 检查是否接近目标，接近目标时不应用优先项，因为目标附近可能存在障碍物
  // If near the goal, don't apply the preferential term since the goal is near obstacles
  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
    near_goal = true;
  }

  // 创建原始成本和斥力成本的数组
  auto && raw_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  raw_cost.fill(0.0f);
  auto && repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  repulsive_cost.fill(0.0f);

  // 获取轨迹长度
  const size_t traj_len = data.trajectories.x.shape(1); //表示每个轨迹有多少个点，即轨迹的长度
  // 是否所有轨迹都与障碍物碰撞
  bool all_trajectories_collide = true;

  // 遍历每条轨迹
  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
    bool trajectory_collide = false; // 该轨迹是否与障碍物碰撞
    float traj_cost = 0.0f; // 轨迹成本
    const auto & traj = data.trajectories;
    CollisionCost pose_cost; // 当前姿态的碰撞成本

    // 遍历轨迹中的每个点
    for (size_t j = 0; j < traj_len; j++) {
      pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), traj.yaws(i, j));
      if (pose_cost.cost < 1.0f) {continue;}  // 在自由空间中

      if (inCollision(pose_cost.cost)) {
        // 如果姿态在碰撞中，则将该轨迹标记为碰撞
        trajectory_collide = true;
        break;
      }

      // 如果膨胀层不存在，则无法处理排斥
      if (inflation_radius_ == 0.0f || inflation_scale_factor_ == 0.0f) {
        continue;
      }

      // 计算到障碍物的距离
      const float dist_to_obj = distanceToObstacle(pose_cost);

      // 让接近碰撞的轨迹点受到严重惩罚
      if (dist_to_obj < collision_margin_distance_) {
        traj_cost += (collision_margin_distance_ - dist_to_obj);
      } else if (!near_goal) {  // 一般而言，更靠近障碍物的轨迹要受到排斥
        repulsive_cost[i] += (inflation_radius_ - dist_to_obj);
      }
    }

    // 如果轨迹没有碰撞，则将所有轨迹碰撞标志设为 false
    if (!trajectory_collide) {all_trajectories_collide = false;}
    raw_cost[i] = trajectory_collide ? collision_cost_ : traj_cost;
  }

  // 计算最终成本，将其加入到数据的成本中
  data.costs += xt::pow(
    (critical_weight_ * raw_cost) +
    (repulsion_weight_ * repulsive_cost / traj_len),
    power_);

  // 设置失败标志，如果所有轨迹都与障碍物碰撞，则将失败标志设为 true
  data.fail_flag = all_trajectories_collide;
}

/**
  * @brief Checks if cost represents a collision
  * @param cost Costmap cost
  * @return bool if in collision
  */
bool ObstaclesCritic::inCollision(float cost) const
{
  // 首先检查costmap是否追踪未知区域
  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  // 根据传入的cost值判断是否处于碰撞状态
  switch (static_cast<unsigned char>(cost)) {
    using namespace nav2_costmap_2d; // 引入nav2_costmap_2d命名空间，简化代码

    case (LETHAL_OBSTACLE): // 致命障碍
      // 如果遇到致命障碍，则认为处于碰撞状态
      return true;

    case (INSCRIBED_INFLATED_OBSTACLE): // 膨胀障碍内部
      // 如果考虑到机器人的足迹，则不认为是碰撞，否则认为是碰撞
      return consider_footprint_ ? false : true;

    case (NO_INFORMATION): // 无信息区域
      // 如果costmap追踪未知区域且当前区域没有信息，则不认为是碰撞，否则认为是碰撞
      return is_tracking_unknown ? false : true;
  }

  // 如果以上条件都不满足，则认为不处于碰撞状态
  return false;
}

CollisionCost ObstaclesCritic::costAtPose(float x, float y, float theta)
{
  // 创建 CollisionCost 结构
  CollisionCost collision_cost;
  // 获取碰撞成本的引用，便于后续修改
  float & cost = collision_cost.cost;
  // 默认不使用足迹
  collision_cost.using_footprint = false;
  unsigned int x_i, y_i;
  
  // 将世界坐标转换为地图坐标
  if (!collision_checker_.worldToMap(x, y, x_i, y_i)) {
    // 如果转换失败，则设置成本为未知
    cost = nav2_costmap_2d::NO_INFORMATION;
    return collision_cost;
  }

  // 获取指定点的碰撞成本
  cost = collision_checker_.pointCost(x_i, y_i);

  // 如果考虑了足迹并且点的成本大于或等于可能的最大内切成本
  if (consider_footprint_ &&
    (cost >= possibly_inscribed_cost_ || possibly_inscribed_cost_ < 1.0f))
  {
    // 使用足迹计算点的成本
    cost = static_cast<float>(collision_checker_.footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint()));
    // 设置使用了足迹的标志为 true
    collision_cost.using_footprint = true;
  }

  // 返回计算得到的碰撞成本和足迹使用标志
  return collision_cost;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ObstaclesCritic,
  mppi::critics::CriticFunction)

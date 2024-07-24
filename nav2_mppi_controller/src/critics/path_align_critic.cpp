// Copyright (c) 2023 Open Navigation LLC
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

#include "nav2_mppi_controller/critics/path_align_critic.hpp"

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::critics
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void PathAlignCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0);

  getParam(max_path_occupancy_ratio_, "max_path_occupancy_ratio", 0.07);
  getParam(offset_from_furthest_, "offset_from_furthest", 20);
  getParam(trajectory_point_step_, "trajectory_point_step", 4);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5);
  getParam(use_path_orientations_, "use_path_orientations", false);

  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
    power_, weight_);
}

void PathAlignCritic::score(CriticData & data)
{
  // Don't apply close to goal, let the goal critics take over
  // 如果评分器未启用或已经接近目标点，则不进行评分，让goal critics接管。
  if (!enabled_ ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  // Don't apply when first getting bearing w.r.t. the path。"当首次相对于路径获取方向时，不适用"
  // 首次相对于路径获取方向时，如果还未达到预设的路径点，则不进行评分。
  utils::setPathFurthestPointIfNotSet(data);
  const size_t path_segments_count = *data.furthest_reached_path_point;  // up to furthest only
  if (path_segments_count < offset_from_furthest_) {
    return;
  }

  // Don't apply when dynamic obstacles are blocking significant proportions of the local path
  // 如果动态障碍物阻塞了局部路径的【大部分】，则跳过评分。
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  const size_t closest_initial_path_point = utils::findPathTrajectoryInitialPoint(data);
  unsigned int invalid_ctr = 0;
  const float range = *data.furthest_reached_path_point - closest_initial_path_point;
  for (size_t i = closest_initial_path_point; i < *data.furthest_reached_path_point; i++) {
    if (!(*data.path_pts_valid)[i]) {invalid_ctr++;}
    if (static_cast<float>(invalid_ctr) / range > max_path_occupancy_ratio_ && invalid_ctr > 2) {
      return;
    }
  }

// 提取路径点并获取轨迹的大小。
  const auto P_x = xt::view(data.path.x, xt::range(_, -1));  // path points
  const auto P_y = xt::view(data.path.y, xt::range(_, -1));  // path points
  const auto P_yaw = xt::view(data.path.yaws, xt::range(_, -1));  // path points

  const size_t batch_size = data.trajectories.x.shape(0);
  const size_t time_steps = data.trajectories.x.shape(1);
  auto && cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});

  // Find integrated distance in the path
  // 计算路径上的积分距离，为后续找到最近路径点提供依据。
  std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
  float dx = 0.0f, dy = 0.0f;
  for (unsigned int i = 1; i != path_segments_count; i++) {
    dx = P_x(i) - P_x(i - 1);
    dy = P_y(i) - P_y(i - 1);
    float curr_dist = sqrtf(dx * dx + dy * dy);
    path_integrated_distances[i] = path_integrated_distances[i - 1] + curr_dist;
  }

// 对于每一条轨迹，计算其与路径的平均对齐成本。
  float traj_integrated_distance = 0.0f; // 每条轨迹的积分距离
  float summed_path_dist = 0.0f, dyaw = 0.0f; // 路径上累积的距离和方向差
  float num_samples = 0.0f;  // 有效样本数
  float Tx = 0.0f, Ty = 0.0f; // 轨迹点的临时坐标
  size_t path_pt = 0; // 路径点索引
  for (size_t t = 0; t < batch_size; ++t) {
     // 重置每条轨迹的积分距离和累积距离。
    traj_integrated_distance = 0.0f;
    summed_path_dist = 0.0f;
    num_samples = 0.0f;
     // 获取当前轨迹的x和y坐标
    const auto T_x = xt::view(data.trajectories.x, t, xt::all());
    const auto T_y = xt::view(data.trajectories.y, t, xt::all());
     // 遍历轨迹上的点
    for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
       // 计算当前点和前一个点之间的距离，累加到轨迹的积分距离上
      Tx = T_x(p);
      Ty = T_y(p);
      dx = Tx - T_x(p - trajectory_point_step_);
      dy = Ty - T_y(p - trajectory_point_step_);
      traj_integrated_distance += sqrtf(dx * dx + dy * dy);
       // 找到与当前积分距离最接近的路径点
      path_pt = utils::findClosestPathPt(
        path_integrated_distances, traj_integrated_distance, path_pt);

      // The nearest path point to align to needs to be not in collision, else
      // let the obstacle critic take over in this region due to dynamic obstacles
      // 需要对齐的最近路径点不能发生碰撞，否则，由于动态障碍物，让obstacle critic在这个区域接管。
        // 如果找到的路径点有效，则累积计算距离和方向差。
      if ((*data.path_pts_valid)[path_pt]) {
        // 计算轨迹点与路径点之间的距离
        dx = P_x(path_pt) - Tx;
        dy = P_y(path_pt) - Ty;
        num_samples += 1.0f;
         // 如果考虑路径的方向
        if (use_path_orientations_) {
            // 计算方向差
          const auto T_yaw = xt::view(data.trajectories.yaws, t, xt::all());
          dyaw = angles::shortest_angular_distance(P_yaw(path_pt), T_yaw(p));
           // 累积距离和方向差的平方根
          summed_path_dist += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
        } else {
          // 只累积距离
          summed_path_dist += sqrtf(dx * dx + dy * dy);
        }
      }
    }

    // 如果有有效样本，则计算平均成本。
    if (num_samples > 0) {
      cost[t] = summed_path_dist / num_samples;
    } else {
      cost[t] = 0.0f;// 无有效样本时成本为0。
    }
  }
// 最后，根据权重和幂次调整成本。
  data.costs += xt::pow(std::move(cost) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAlignCritic,
  mppi::critics::CriticFunction)

// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_

#include <algorithm>
#include <chrono>
#include <string>
#include <limits>
#include <memory>
#include <vector>

#include <xtensor/xarray.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "angles/angles.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_core/goal_checker.hpp"

#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav2_mppi_controller/critic_data.hpp"

namespace mppi::utils
{
using xt::evaluation_strategy::immediate;

/**
 * @brief Convert data into pose
 * @param x X position
 * @param y Y position
 * @param z Z position
 * @return Pose object
 */
inline geometry_msgs::msg::Pose createPose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  return pose;
}

/**
 * @brief Convert data into scale
 * @param x X scale
 * @param y Y scale
 * @param z Z scale
 * @return Scale object
 */
inline geometry_msgs::msg::Vector3 createScale(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}

/**
 * @brief Convert data into color
 * @param r Red component
 * @param g Green component
 * @param b Blue component
 * @param a Alpha component (transparency)
 * @return Color object
 */
inline std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

/**
 * @brief Convert data into a Maarker
 * @param id Marker ID
 * @param pose Marker pose
 * @param scale Marker scale
 * @param color Marker color
 * @param frame Reference frame to use
 * @return Visualization Marker
 */
inline visualization_msgs::msg::Marker createMarker(
  int id, const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color, const std::string & frame_id, const std::string & ns)
{
  using visualization_msgs::msg::Marker;
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time(0, 0);
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;

  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;
  return marker;
}

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */
inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float wz, const builtin_interfaces::msg::Time & stamp, const std::string & frame)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame;
  twist.header.stamp = stamp;
  twist.twist.linear.x = vx;
  twist.twist.angular.z = wz;

  return twist;
}

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param vy Y velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */
inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float vy, float wz, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame)
{
  auto twist = toTwistStamped(vx, wz, stamp, frame);
  twist.twist.linear.y = vy;

  return twist;
}

/**
 * @brief Convert path to a tensor
 * @param path Path to convert
 * @return Path tensor
 */
inline models::Path toTensor(const nav_msgs::msg::Path & path)
{
  auto result = models::Path{};
  result.reset(path.poses.size());

  for (size_t i = 0; i < path.poses.size(); ++i) {
    result.x(i) = path.poses[i].pose.position.x;
    result.y(i) = path.poses[i].pose.position.y;
    result.yaws(i) = tf2::getYaw(path.poses[i].pose.orientation);
  }

  return result;
}

/**
 * @brief Check if the robot pose is within the Goal Checker's tolerances to goal
 * @param global_checker Pointer to the goal checker
 * @param robot Pose of robot
 * @param path Path to retreive goal pose from
 * @return bool If robot is within goal checker tolerances to the goal
 */
inline bool withinPositionGoalTolerance(
  nav2_core::GoalChecker * goal_checker,
  const geometry_msgs::msg::Pose & robot,
  const models::Path & path)
{
  const auto goal_idx = path.x.shape(0) - 1;
  const auto goal_x = path.x(goal_idx);
  const auto goal_y = path.y(goal_idx);

  if (goal_checker) {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    goal_checker->getTolerances(pose_tolerance, velocity_tolerance);

    const auto pose_tolerance_sq = pose_tolerance.position.x * pose_tolerance.position.x;

    auto dx = robot.position.x - goal_x;
    auto dy = robot.position.y - goal_y;

    auto dist_sq = dx * dx + dy * dy;

    if (dist_sq < pose_tolerance_sq) {
      return true;
    }
  }

  return false;
}

/**
 * @brief Check if the robot pose is within tolerance to the goal
 * @param pose_tolerance Pose tolerance to use
 * @param robot Pose of robot
 * @param path Path to retreive goal pose from
 * @return bool If robot is within tolerance to the goal
 */

/**
 * @brief 检查机器人姿态是否在与目标的容差范围内
 * @param pose_tolerance 使用的姿态容差
 * @param robot 机器人的姿态
 * @param path 用于获取目标姿态的路径
 * @return bool 如果机器人在与目标的容差范围内，则为true
 */
inline bool withinPositionGoalTolerance(
  float pose_tolerance,
  const geometry_msgs::msg::Pose & robot,
  const models::Path & path)
{
  // 获取路径的最后一个索引和目标位置
  const auto goal_idx = path.x.shape(0) - 1;
  const auto goal_x = path.x(goal_idx);
  const auto goal_y = path.y(goal_idx);

  // 计算姿态容差的平方
  const auto pose_tolerance_sq = pose_tolerance * pose_tolerance;

  // 计算机器人与目标位置的距离的平方
  auto dx = robot.position.x - goal_x;
  auto dy = robot.position.y - goal_y;
  auto dist_sq = dx * dx + dy * dy;

  // 如果距离的平方小于姿态容差的平方，则返回true，表示在容差范围内
  if (dist_sq < pose_tolerance_sq) {
    return true;
  }

  // 否则返回false，表示不在容差范围内
  return false;
}

/**
  * @brief normalize
  * Normalizes the angle to be -M_PI circle to +M_PI circle
  * It takes and returns radians.
  * @param angles Angles to normalize
  * @return normalized angles
  */

/**
 * @brief normalize
 * 将角度规范化到 -π 到 +π 的圆周范围内
 * 输入和输出都是以弧度表示的角度。
 * @param angles 要规范化的角度
 * @return 规范化后的角度
 */
template<typename T>
auto normalize_angles(const T & angles)
{
  // 使用 fmod 函数将角度限制在 -π 到 +π 的范围内
  auto && theta = xt::eval(xt::fmod(angles + M_PI, 2.0 * M_PI)); //计算 (angles + M_PI) % (2.0 * M_PI)，即将 angles 加上 π，然后对 2.0 * M_PI 取模。这样可以确保结果始终在 -π 到 +π 的范围内
  
  // 使用 where 函数将小于等于 0 的角度调整为正数
  return xt::eval(xt::where(theta <= 0.0, theta + M_PI, theta - M_PI));
}


/**
  * @brief shortest_angular_distance
  *
  * Given 2 angles, this returns the shortest angular
  * difference.  The inputs and ouputs are of course radians.
  *
  * The result
  * would always be -pi <= result <= pi.  Adding the result
  * to "from" will always get you an equivelent angle to "to".
  * @param from Start angle
  * @param to End angle
  * @return Shortest distance between angles
  */

/**
 * @brief shortest_angular_distance
 *
 * 给定两个角度，这个函数返回最短的角度差。输入和输出都是弧度。
 *
 * 结果始终满足 -π <= result <= π。将结果添加到 "from" 上将始终得到与 "to" 等价的角度。
 * @param from 起始角度
 * @param to 结束角度
 * @return 角度之间的最短距离
 */
template<typename F, typename T>
auto shortest_angular_distance(
  const F & from,
  const T & to)
{
  return normalize_angles(to - from);
}


/**
 * @brief Evaluate furthest point idx of data.path which is
 * nearset to some trajectory in data.trajectories
 * @param data Data to use
 * @return Idx of furthest path point reached by a set of trajectories
 */

/**
 * @brief 计算在data.path中，最接近data.trajectories中某条轨迹的最远点的索引
 * @param data 要使用的数据
 * @return 由一组轨迹所能到达的路径点的最远索引
 */
inline size_t findPathFurthestReachedPoint(const CriticData & data)
{
  // 获取最后一个点的x坐标，并增加一个新的轴（维度）
  const auto traj_x = xt::view(data.trajectories.x, xt::all(), -1, xt::newaxis());
  // 获取最后一个点的y坐标，并增加一个新的轴（维度）
  const auto traj_y = xt::view(data.trajectories.y, xt::all(), -1, xt::newaxis());

  // 计算路径点与轨迹点在x方向上的差值
  const auto dx = data.path.x - traj_x;
  // 计算路径点与轨迹点在y方向上的差值
  const auto dy = data.path.y - traj_y;

  // 计算每对点之间的欧氏距离的平方
  const auto dists = dx * dx + dy * dy;

  // 初始化最远点索引和路径上最近点的索引，以及最近点的距离
  size_t max_id_by_trajectories = 0, min_id_by_path = 0;
  float min_distance_by_path = std::numeric_limits<float>::max();
  float cur_dist = 0.0f;

  // 遍历所有轨迹点
  for (size_t i = 0; i < dists.shape(0); i++) {
    // 对于每条轨迹，重置最近点的索引和距离
    min_id_by_path = 0;

    // 遍历所有路径点
    for (size_t j = 0; j < dists.shape(1); j++) {
      // 获取当前点的距离
      cur_dist = dists(i, j);
      // 如果当前点的距离小于当前最小距离，则更新最小距离和最近点索引
      if (cur_dist < min_distance_by_path) {
        min_distance_by_path = cur_dist;
        min_id_by_path = j;
      }
    }
    // 更新最远到达点的索引
    max_id_by_trajectories = std::max(max_id_by_trajectories, min_id_by_path);
  }
  // 返回最远到达点的索引
  return max_id_by_trajectories;
}

/**
 * @brief Evaluate closest point idx of data.path which is
 * nearset to the start of the trajectory in data.trajectories
 * @param data Data to use
 * @return Idx of closest path point at start of the trajectories
 */

/**
 * @brief 计算在data.path中，最接近data.trajectories轨迹起点的路径点的索引
 * @param data 要使用的数据
 * @return 最接近轨迹起始点的路径点的索引
 */
inline size_t findPathTrajectoryInitialPoint(const CriticData & data)
{
  // 假定所有轨迹从相同的初始条件开始，取轨迹的第一个点
  const auto dx = data.path.x - data.trajectories.x(0, 0); // 计算路径点与轨迹起点在x方向的差值
  const auto dy = data.path.y - data.trajectories.y(0, 0); // 计算路径点与轨迹起点在y方向的差值
  // 计算每个路径点到轨迹起点的欧氏距离的平方
  const auto dists = dx * dx + dy * dy;

  // 初始化最小距离为最大浮点数，确保任何实际距离都会小于这个值
  float min_distance_by_path = std::numeric_limits<float>::max();
  size_t min_id = 0; // 用于记录最近点的索引
  // 遍历所有路径点
  for (size_t j = 0; j < dists.shape(0); j++) {
    // 如果当前点到起点的距离小于已记录的最小距离
    if (dists(j) < min_distance_by_path) {
      // 更新最小距离和最近点索引
      min_distance_by_path = dists(j);
      min_id = j;
    }
  }

  // 返回最近点的索引
  return min_id;
}

/**
 * @brief evaluate path furthest point if it is not set
 * @param data Data to use
 */

/**
 * @brief 如果尚未设置，评估并设置路径中最远到达的点
 * @param data 用于操作的数据
 */
inline void setPathFurthestPointIfNotSet(CriticData & data)
{
  if (!data.furthest_reached_path_point) {
    data.furthest_reached_path_point = findPathFurthestReachedPoint(data);
  }
}

/**
 * @brief evaluate path costs
 * @param data Data to use
 */

/**
 * @brief 评估路径成本
 * @param data 要使用的数据
 * @param costmap_ros 成本地图的共享指针，用于查询路径点的成本
 * @note *data.path_pts_valid 判断有效性
 */
inline void findPathCosts(
  CriticData & data,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // 获取成本地图的原始指针
  auto * costmap = costmap_ros->getCostmap();
  unsigned int map_x, map_y; // 用于存储路径点在地图上的坐标
  // 计算路径段的数量（路径点的数量减1）
  const size_t path_segments_count = data.path.x.shape(0) - 1;
  // 初始化路径点有效性的向量，初始假设所有路径段无效
  data.path_pts_valid = std::vector<bool>(path_segments_count, false);
  // 遍历所有路径段
  for (unsigned int idx = 0; idx < path_segments_count; idx++) {
    // 获取当前路径点的坐标
    const auto path_x = data.path.x(idx);
    const auto path_y = data.path.y(idx);
    // 将世界坐标转换为地图坐标，如果转换失败，则跳过当前路径点
    if (!costmap->worldToMap(path_x, path_y, map_x, map_y)) {
      (*data.path_pts_valid)[idx] = false;
      continue;
    }

    // 根据路径点在地图上的坐标，获取其成本
    switch (costmap->getCost(map_x, map_y)) {
      using namespace nav2_costmap_2d; // 使用nav2_costmap_2d命名空间中的常量
      case (LETHAL_OBSTACLE): // 如果是致命障碍物
        (*data.path_pts_valid)[idx] = false;
        continue;
      case (INSCRIBED_INFLATED_OBSTACLE): // 如果是膨胀障碍物
        (*data.path_pts_valid)[idx] = false;
        continue;
      case (NO_INFORMATION): // 如果是未知区域
        // 查询是否追踪未知区域
        const bool is_tracking_unknown =
          costmap_ros->getLayeredCostmap()->isTrackingUnknown();
        // 根据是否追踪未知区域设置路径点的有效性
        (*data.path_pts_valid)[idx] = is_tracking_unknown ? true : false;
        continue;
    }

    // 如果路径点不是上述特殊情况，则认为是有效的
    (*data.path_pts_valid)[idx] = true;
  }
}

/**
 * @brief evaluate path costs if it is not set
 * @param data Data to use
 */
inline void setPathCostsIfNotSet(
  CriticData & data,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  if (!data.path_pts_valid) {
    findPathCosts(data, costmap_ros);
  }
}

/**
 * @brief evaluate angle from pose (have angle) to point (no angle)
 * @param pose pose
 * @param point_x Point to find angle relative to X axis
 * @param point_y Point to find angle relative to Y axis
 * @param forward_preference If reversing direction is valid
 * @return Angle between two points
 */

/**
 * @brief 计算从pose（具有角度）到point（无角度）的角度
 * 
 * 该函数计算从给定位姿到指定点的相对角度。如果机器人没有前进方向的偏好，则返回两个角度中较小的一个，即当前航向或航向加上180度的角度差。
 * 如果机器人有前进方向的偏好，则只计算相对于当前航向的角度差。
 *
 * @param pose 机器人的位姿信息，包含位置和朝向
 * @param point_x 目标点的X轴坐标
 * @param point_y 目标点的Y轴坐标
 * @param forward_preference 机器人是否有前进方向的偏好，即是否允许倒车
 * @return 返回两点之间的角度差
 */
inline float posePointAngle(
  const geometry_msgs::msg::Pose & pose, double point_x, double point_y, bool forward_preference)
{
  // 从位姿中提取机器人的位置和航向角
  float pose_x = pose.position.x;
  float pose_y = pose.position.y;
  float pose_yaw = tf2::getYaw(pose.orientation); // 提取航向角

  // 计算目标点相对于机器人当前位置的角度（极坐标角度）
  float yaw = atan2f(point_y - pose_y, point_x - pose_x);

  // 如果机器人没有前进方向的偏好，则计算相对于当前航向的角度差，或者航向加上180度的角度差中较小的一个
  if (!forward_preference) {
    return std::min(
      fabs(angles::shortest_angular_distance(yaw, pose_yaw)),  // 计算相对于当前航向的角度差
      fabs(angles::shortest_angular_distance(yaw, angles::normalize_angle(pose_yaw + M_PI))));  // 计算相对于航向加180度的角度差
  }

  // 如果机器人有前进方向的偏好，则只计算相对于当前航向的角度差
  return fabs(angles::shortest_angular_distance(yaw, pose_yaw));
}

/**
 * @brief Apply Savisky-Golay filter to optimal trajectory
 * @param control_sequence Sequence to apply filter to
 * @param control_history 最近的一组控制历史，用于边缘情况处理。Recent set of controls for edge-case handling
 * @param Settings Settings to use
 */
inline void savitskyGolayFilter(
  models::ControlSequence & control_sequence,
  std::array<mppi::models::Control, 4> & control_history,
  const models::OptimizerSettings & settings)
{
  // Savitzky-Golay Quadratic, 9-point Coefficients。Savitzky-Golay 二次型，9 点系数
  xt::xarray<float> filter = {-21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0}; 
  filter /= 231.0; //滤波器系数经过归一化，确保总和为 1，用于保持信号的能量不变

  const unsigned int num_sequences = control_sequence.vx.shape(0) - 1;

  // Too short to smooth meaningfully。 太短无法有效平滑
  if (num_sequences < 20) {
    return;
  }

  auto applyFilter = [&](const xt::xarray<float> & data) -> float {
      return xt::sum(data * filter, {0}, immediate)();
    };

  auto applyFilterOverAxis =
    [&](xt::xtensor<float, 1> & sequence,
      const float hist_0, const float hist_1, const float hist_2, const float hist_3) -> void
    {
      unsigned int idx = 0;
       // 使用历史数据和序列中的当前及后续数据点应用滤波器
      sequence(idx) = applyFilter(
      {
        hist_0,
        hist_1,
        hist_2,
        hist_3,
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      // 索引递增，对序列中的下一个数据点应用滤波器
      idx++;
      // 重复此过程，直到处理完序列中的所有数据点
      sequence(idx) = applyFilter(
      {
        hist_1,
        hist_2,
        hist_3,
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
      {
        hist_2,
        hist_3,
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
      {
        hist_3,
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      // 对序列中间部分的数据点应用滤波器
      for (idx = 4; idx != num_sequences - 4; idx++) {
        sequence(idx) = applyFilter(
        {
          sequence(idx - 4),
          sequence(idx - 3),
          sequence(idx - 2),
          sequence(idx - 1),
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 2),
          sequence(idx + 3),
          sequence(idx + 4)});
      }

      // 对序列末尾的数据点应用滤波器，使用重复的最后一个数据点作为历史数据
      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 3)});

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 2),
        sequence(idx + 2)});

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 1),
        sequence(idx + 1),
        sequence(idx + 1)});

      // 最后一个数据点使用自身作为历史数据
      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx),
        sequence(idx),
        sequence(idx),
        sequence(idx)});
    };

  // Filter trajectories
  // 对控制序列 vx, vy, wz 应用滤波器
  applyFilterOverAxis(
    control_sequence.vx, control_history[0].vx,
    control_history[1].vx, control_history[2].vx, control_history[3].vx);
  applyFilterOverAxis(
    control_sequence.vy, control_history[0].vy,
    control_history[1].vy, control_history[2].vy, control_history[3].vy);
  applyFilterOverAxis(
    control_sequence.wz, control_history[0].wz,
    control_history[1].wz, control_history[2].wz, control_history[3].wz);

  // Update control history
  // 更新控制历史记录
  unsigned int offset = settings.shift_control_sequence ? 1 : 0;
  control_history[0] = control_history[1];
  control_history[1] = control_history[2];
  control_history[2] = control_history[3];
  control_history[3] = {
    control_sequence.vx(offset),
    control_sequence.vy(offset),
    control_sequence.wz(offset)};
}

/**
 * @brief Find the iterator of the first pose at which there is an inversion on the path,
 * @param path to check for inversion
 * @return the first point after the inversion found in the path
 */
/**
 * @brief 查找路径上第一个倒置点的迭代器。当路径中出现方向突然反转的情况时，需要找到并处理这种倒置点，以确保路径的连续性和可行性
 * @param path 要检查倒置的路径
 * @return 在路径中找到的倒置点之后的第一个点
 */
inline unsigned int findFirstPathInversion(nav_msgs::msg::Path & path)
{
  // 至少需要3个位置点才可能存在倒置
  if (path.poses.size() < 3) {
    return path.poses.size();
  }

  // 遍历路径以确定路径倒置的位置
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // 我们有两个向量用于点积 OA 和 AB。确定向量。
    float oa_x = path.poses[idx].pose.position.x -
      path.poses[idx - 1].pose.position.x;
    float oa_y = path.poses[idx].pose.position.y -
      path.poses[idx - 1].pose.position.y;
      
    float ab_x = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    float ab_y = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;

    // 使用点积检查路径中是否存在尖角。
    float dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    // 如果点积小于0，则表示找到了倒置点
    if (dot_product < 0.0) {
      return idx + 1;
    }
  }

  // 如果没有找到倒置点，返回路径位置点的数量
  return path.poses.size();
}


/**
 * @brief Find and remove poses after the first inversion in the path
 * @param path to check for inversion
 * @return The location of the inversion, return 0 if none exist
 */
/**
 * @brief 在路径中找到并移除第一个倒置点之后的所有位置
 * @param path 要检查倒置的路径
 * @return 倒置点的位置，如果不存在则返回0
 */
inline unsigned int removePosesAfterFirstInversion(nav_msgs::msg::Path & path)
{
  nav_msgs::msg::Path cropped_path = path; // 创建路径的副本
  // 调用findFirstPathInversion函数找到第一个倒置点
  const unsigned int first_after_inversion = findFirstPathInversion(cropped_path);
  // 如果没有找到倒置点，返回0
  if (first_after_inversion == path.poses.size()) {
    return 0u;
  }

  // 从第一个倒置点开始移除路径中的位置
  cropped_path.poses.erase(
    cropped_path.poses.begin() + first_after_inversion, cropped_path.poses.end());
  path = cropped_path; // 更新原路径为剪裁后的路径
  return first_after_inversion; // 返回倒置点的位置
}


/**
 * @brief Compare to trajectory points to find closest path point along integrated distances
 * @param vec Vect to check
 * @return dist Distance to look for
 */

/**
 * @brief 比较轨迹点以找到沿积分距离最接近的路径点
 * @param vec 要检查的向量，预期是按积分距离排序的
 * @param dist 要查找的距离
 * @param init 查找开始的初始索引，默认为0
 * @return 返回与给定距离最接近的路径点的索引
 */
inline size_t findClosestPathPt(const std::vector<float> & vec, float dist, size_t init = 0)
{
   // 使用std::lower_bound在vec中从init开始查找第一个不小于dist的元素的迭代器
  auto iter = std::lower_bound(vec.begin() + init, vec.end(), dist);

  // 如果找到的元素就是开始查找的位置，表示没有找到更接近的点，直接返回0
  if (iter == vec.begin() + init) {
    return 0;
  }

   // 如果dist与前一个元素的差小于dist与当前找到的元素的差，则返回前一个元素的索引
  if (dist - *(iter - 1) < *iter - dist) {
    return iter - 1 - vec.begin();
  }

    // 否则，返回当前找到的元素的索引
  return iter - vec.begin();
}

}  // namespace mppi::utils

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_

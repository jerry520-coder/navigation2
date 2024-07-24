// Copyright (c) 2019 Intel Corporation
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
//
// Modified by: Shivang Patel (shivaang14@gmail.com)

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/line_iterator.hpp"

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{

template<typename CostmapT>
FootprintCollisionChecker<CostmapT>::FootprintCollisionChecker()
: costmap_(nullptr)
{
}

template<typename CostmapT>
FootprintCollisionChecker<CostmapT>::FootprintCollisionChecker(
  CostmapT costmap)
: costmap_(costmap)
{
}

template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::footprintCost(const Footprint footprint)
{
  // now we really have to lay down the footprint in the costmap_ grid。现在我们需要真正地将足迹放置在costmap_网格上
  unsigned int x0, x1, y0, y1;
  double footprint_cost = 0.0;

  // get the cell coord of the first point。  获取第一个点的网格坐标
  if (!worldToMap(footprint[0].x, footprint[0].y, x0, y0)) {
    return static_cast<double>(LETHAL_OBSTACLE); // 如果第一个点在地图之外，返回致命障碍的代价
  }

  // cache the start to eliminate a worldToMap call。缓存起点的坐标，以避免重复调用worldToMap
  unsigned int xstart = x0;
  unsigned int ystart = y0;

  // we need to rasterize each line in the footprint。我们需要对足迹中的每一条线进行光栅化处理
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // get the cell coord of the second point  // 获取第二个点的网格坐标
    if (!worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      return static_cast<double>(LETHAL_OBSTACLE);
    }

 // 计算当前线段的代价，并与当前最高代价进行比较
    footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);

    // the second point is next iteration's first point。第二个点作为下一次迭代的第一个点
    x0 = x1;
    y0 = y1;

    // if in collision, no need to continue // 如果当前线段的代价为致命障碍，不需要继续计算
    if (footprint_cost == static_cast<double>(LETHAL_OBSTACLE)) {
      return footprint_cost;
    }
  }

  // we also need to connect the first point in the footprint to the last point
  // the last iteration's x1, y1 are the last footprint point's coordinates

  // 还需要连接足迹的第一个点和最后一个点
  // 上一次迭代的x1, y1是最后一个足迹点的坐标
  return std::max(lineCost(xstart, x1, ystart, y1), footprint_cost);
}

template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::lineCost(int x0, int x1, int y0, int y1) const
{
  double line_cost = 0.0;
  double point_cost = -1.0;

// 使用nav2_util::LineIterator遍历从(x0, y0)到(x1, y1)的所有点
  for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = pointCost(line.getX(), line.getY());   // Score the current point // 计算当前点的代价

    // if in collision, no need to continue  // 如果当前点的代价为致命障碍，不需要继续计算
    if (point_cost == static_cast<double>(LETHAL_OBSTACLE)) {
      return point_cost;
    }

  // 更新当前线段的最大代价
    if (line_cost < point_cost) {
      line_cost = point_cost;
    }
  }

  return line_cost;
}

template<typename CostmapT>
bool FootprintCollisionChecker<CostmapT>::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my)
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::pointCost(int x, int y) const
{
  return costmap_->getCost(x, y);
}

template<typename CostmapT>
void FootprintCollisionChecker<CostmapT>::setCostmap(CostmapT costmap)
{
  costmap_ = costmap;
}

template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::footprintCostAtPose(
  double x, double y, double theta, const Footprint footprint)
{
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  Footprint oriented_footprint; // 存储变换后的足迹点
  for (unsigned int i = 0; i < footprint.size(); ++i) {
    geometry_msgs::msg::Point new_pt;
    new_pt.x = x + (footprint[i].x * cos_th - footprint[i].y * sin_th);
    new_pt.y = y + (footprint[i].x * sin_th + footprint[i].y * cos_th);
    oriented_footprint.push_back(new_pt); // 将变换后的点添加到新的足迹列表中
  }

  return footprintCost(oriented_footprint);
}

// declare our valid template parameters
template class FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>;
template class FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>;

}  // namespace nav2_costmap_2d

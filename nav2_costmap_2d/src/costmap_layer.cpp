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
 *         David V. Lu!!
 *********************************************************************/

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <stdexcept>
#include <algorithm>

namespace nav2_costmap_2d
{

void CostmapLayer::touch(
  double x, double y, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  *min_x = std::min(x, *min_x);
  *min_y = std::min(y, *min_y);
  *max_x = std::max(x, *max_x);
  *max_y = std::max(y, *max_y);
}

void CostmapLayer::matchSize()
{
  Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
    master->getOriginX(), master->getOriginY());
}

void CostmapLayer::clearArea(int start_x, int start_y, int end_x, int end_y, bool invert)
{
  current_ = false;
  unsigned char * grid = getCharMap();

   // 遍历代价图中的每个格子
  for (int x = 0; x < static_cast<int>(getSizeInCellsX()); x++) {
    bool xrange = x > start_x && x < end_x;  // 判断x坐标是否在指定的x范围内

    for (int y = 0; y < static_cast<int>(getSizeInCellsY()); y++) {
      // 判断y坐标是否在指定的y范围内，并结合invert参数决定是否清除当前格子
      if ((xrange && y > start_y && y < end_y) == invert) {
        continue;// 如果当前格子在操作区域内（或外，取决于invert），则跳过当前循环
      }
      int index = getIndex(x, y);// 计算当前格子在数组中的索引
      if (grid[index] != NO_INFORMATION) { // 如果当前格子的信息不是NO_INFORMATION
        grid[index] = NO_INFORMATION; // 将当前格子设置为无信息状态
      }
    }
  }
}

void CostmapLayer::addExtraBounds(double mx0, double my0, double mx1, double my1)
{
  extra_min_x_ = std::min(mx0, extra_min_x_);
  extra_max_x_ = std::max(mx1, extra_max_x_);
  extra_min_y_ = std::min(my0, extra_min_y_);
  extra_max_y_ = std::max(my1, extra_max_y_);
  has_extra_bounds_ = true;
}

void CostmapLayer::useExtraBounds(double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!has_extra_bounds_) {
    return;
  }

  *min_x = std::min(extra_min_x_, *min_x);
  *min_y = std::min(extra_min_y_, *min_y);
  *max_x = std::max(extra_max_x_, *max_x);
  *max_y = std::max(extra_max_y_, *max_y);
  extra_min_x_ = 1e6;
  extra_min_y_ = 1e6;
  extra_max_x_ = -1e6;
  extra_max_y_ = -1e6;
  has_extra_bounds_ = false;
}

void CostmapLayer::updateWithMax(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      if (costmap_[it] == NO_INFORMATION) {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION || old_cost < costmap_[it]) {
        master_array[it] = costmap_[it];
      }
      it++;
    }
  }
}

void CostmapLayer::updateWithTrueOverwrite(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  // 如果图层未启用，直接返回
  if (!enabled_) {
    return;
  }

 // 如果costmap_未初始化，抛出异常
  if (costmap_ == nullptr) {
    throw std::runtime_error("Can't update costmap layer: It has't been initialized yet!");
  }

  unsigned char * master = master_grid.getCharMap();// 获取master_grid的字符地图
  unsigned int span = master_grid.getSizeInCellsX(); // 获取master_grid在x方向上的单元格数量

// 遍历指定的更新范围
  for (int j = min_j; j < max_j; j++) {
    unsigned int it = span * j + min_i;
    for (int i = min_i; i < max_i; i++) { // 将当前图层的代价值覆盖到master_grid中
      master[it] = costmap_[it];
      it++;// 移动到下一列
    }
  }
}

void CostmapLayer::updateWithOverwrite(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  unsigned char * master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = span * j + min_i;
    for (int i = min_i; i < max_i; i++) {
      if (costmap_[it] != NO_INFORMATION) {
        master[it] = costmap_[it];
      }
      it++;
    }
  }
}

void CostmapLayer::updateWithAddition(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      if (costmap_[it] == NO_INFORMATION) {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION) {
        master_array[it] = costmap_[it];
      } else {
        int sum = old_cost + costmap_[it];
        if (sum >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          master_array[it] = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
        } else {
          master_array[it] = sum;
        }
      }
      it++;
    }
  }
}
}  // namespace nav2_costmap_2d

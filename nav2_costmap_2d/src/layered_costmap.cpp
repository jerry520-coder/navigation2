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
#include "nav2_costmap_2d/layered_costmap.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "nav2_costmap_2d/footprint.hpp"


using std::vector;

namespace nav2_costmap_2d
{

LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown)
: primary_costmap_(), combined_costmap_(),
  global_frame_(global_frame),
  rolling_window_(rolling_window),
  current_(false),
  minx_(0.0),
  miny_(0.0),
  maxx_(0.0),
  maxy_(0.0),
  bx0_(0),// 格子地图x起始坐标初始化为0
  bxn_(0),
  by0_(0),
  byn_(0),
  initialized_(false),
  size_locked_(false),
  circumscribed_radius_(1.0),
  inscribed_radius_(0.1)
{
  if (track_unknown) {
    primary_costmap_.setDefaultValue(255);// 如果追踪未知区域，设置默认值为255
    combined_costmap_.setDefaultValue(255);// 对于组合地图也设置为255
  } else {
    primary_costmap_.setDefaultValue(0);// 如果不追踪未知区域，设置默认值为0
    combined_costmap_.setDefaultValue(0); // 对于组合地图也设置为0
  }
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0) {
    plugins_.pop_back();
  }
  while (filters_.size() > 0) {
    filters_.pop_back();
  }
}

void LayeredCostmap::addPlugin(std::shared_ptr<Layer> plugin)
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));
  plugins_.push_back(plugin);
}

void LayeredCostmap::addFilter(std::shared_ptr<Layer> filter)
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));
  filters_.push_back(filter);
}

void LayeredCostmap::resizeMap(
  unsigned int size_x, unsigned int size_y, double resolution,
  double origin_x,
  double origin_y,
  bool size_locked)
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));//确保在调整大小期间不会有其他线程修改地图
  size_locked_ = size_locked;  // 设置地图尺寸锁定状态
  primary_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y); // 调整主地图（primary_costmap_）的大小
  combined_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    (*plugin)->matchSize();
  }
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
    filter != filters_.end(); ++filter)
  {
    (*filter)->matchSize();
  }
}

bool LayeredCostmap::isOutofBounds(double robot_x, double robot_y)
{
  unsigned int mx, my;
  return !combined_costmap_.worldToMap(robot_x, robot_y, mx, my);
}

void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)。在此函数的剩余部分加锁，一些插件（例如 VoxelLayer）
  // implement thread unsafe updateBounds() functions.。实现了线程不安全的 updateBounds() 函数。
  std::unique_lock<Costmap2D::mutex_t> lock(*(combined_costmap_.getMutex()));

  // if we're using a rolling buffer costmap...。如果我们使用滚动缓冲代价地图...
  // we need to update the origin using the robot's position。 我们需要使用机器人的位置更新原点。
  if (rolling_window_) {
    double new_origin_x = robot_x - combined_costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - combined_costmap_.getSizeInMetersY() / 2;
    primary_costmap_.updateOrigin(new_origin_x, new_origin_y); // 更新主成本图原点
    combined_costmap_.updateOrigin(new_origin_x, new_origin_y);// 更新组合成本图原点
  }

  // 检查机器人是否超出成本图界限 
  if (isOutofBounds(robot_x, robot_y)) {
    RCLCPP_WARN(
      rclcpp::get_logger("nav2_costmap_2d"),
      "Robot is out of bounds of the costmap!");
  }

  // 如果没有插件和过滤器，则直接返回
  if (plugins_.size() == 0 && filters_.size() == 0) {
    return;
  }

  minx_ = miny_ = std::numeric_limits<double>::max();
  maxx_ = maxy_ = std::numeric_limits<double>::lowest();

  // 遍历所有插件，更新边界
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);

    // 检查边界是否有非法变化
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "nav2_costmap_2d"), "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but " // 非法边界变更
        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
        prev_minx, prev_miny, prev_maxx, prev_maxy,
        minx_, miny_, maxx_, maxy_,
        (*plugin)->getName().c_str());
    }
  }

  // 遍历所有过滤器，更新边界
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
    filter != filters_.end(); ++filter)
  {
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    (*filter)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);


    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "nav2_costmap_2d"), "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "// 非法边界变更
        "is now [tl: (%f, %f), br: (%f, %f)]. The offending filter is %s",
        prev_minx, prev_miny, prev_maxx, prev_maxy,
        minx_, miny_, maxx_, maxy_,
        (*filter)->getName().c_str());
    }
  }

  // 将地图坐标转换为网格坐标，并确保边界有效
  int x0, xn, y0, yn;
  combined_costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  combined_costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  x0 = std::max(0, x0);
  xn = std::min(static_cast<int>(combined_costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(static_cast<int>(combined_costmap_.getSizeInCellsY()), yn + 1);

  RCLCPP_DEBUG(
    rclcpp::get_logger(
      "nav2_costmap_2d"), "Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);


  // 如果更新区域有效，则根据是否有filters_采取不同的更新策略
  if (xn < x0 || yn < y0) {
    return;
  }

  if (filters_.size() == 0) {
    // 如果没有启用过滤器，只需按每个插件的顺序更新成本计算图。If there are no filters enabled just update costmap sequentially by each plugin
    combined_costmap_.resetMap(x0, y0, xn, yn);
    for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
      plugin != plugins_.end(); ++plugin)
    {
      (*plugin)->updateCosts(combined_costmap_, x0, y0, xn, yn);
    }
  } else {
    // Costmap Filters enabled。
    // 1. Update costmap by plugins。
    primary_costmap_.resetMap(x0, y0, xn, yn);
    for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
      plugin != plugins_.end(); ++plugin)
    {
      (*plugin)->updateCosts(primary_costmap_, x0, y0, xn, yn);
    }

    // 2. Copy processed costmap window to a final costmap.。将处理后的代价图窗口复制到最终代价图
    // primary_costmap_ remain to be untouched for further usage by plugins.。primary_costmap_ 保留不变，供插件进一步使用
    if (!combined_costmap_.copyWindow(primary_costmap_, x0, y0, xn, yn, x0, y0)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("nav2_costmap_2d"),
        "Can not copy costmap (%i,%i)..(%i,%i) window",
        x0, y0, xn, yn);
      throw std::runtime_error{"Can not copy costmap"};
    }

    // 3. Apply filters over the plugins in order to make filters' work。在插件上应用过滤器，使过滤器发挥作用
    // not being considered by plugins on next updateMap() calls。插件在下次 updateMap() 调用时不会考虑
    for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
      filter != filters_.end(); ++filter)
    {
      (*filter)->updateCosts(combined_costmap_, x0, y0, xn, yn);
    }
  }

  // 记录更新的格子地图坐标
  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true; // 标记成本图已经初始化完成
}

bool LayeredCostmap::isCurrent()
{
  current_ = true;// 初始化current_变量为true，假设所有层最初都是最新的

   // 遍历插件层列表plugins_
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    // 更新current_的值：
    // 如果当前插件层是最新的，或者如果当前插件层未启用，则保持或更新current_为true
    // 只要有任何一个插件层不是最新的且被启用，current_就会变为false
    current_ = current_ && ((*plugin)->isCurrent() || !(*plugin)->isEnabled());
  }

  // 遍历过滤器层列表filters_
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
    filter != filters_.end(); ++filter)
  {
    current_ = current_ && ((*filter)->isCurrent() || !(*filter)->isEnabled());// 同上，对过滤器层进行相同逻辑的处理
  }
  return current_;// 返回最终的current_值，表明是否所有启用的层都是最新的
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec)
{
  footprint_ = footprint_spec;

  // 计算新足迹的内接圆和外接圆半径
  nav2_costmap_2d::calculateMinAndMaxDistances(
    footprint_spec,
    inscribed_radius_, circumscribed_radius_);

// 通知所有插件足迹已经改变
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->onFootprintChanged();// 调用每个插件的 onFootprintChanged 方法，允许它们根据新的足迹进行适当的响应
  }

  // 通知所有过滤器足迹已经改变
  for (vector<std::shared_ptr<Layer>>::iterator filter = filters_.begin();
    filter != filters_.end();
    ++filter)
  {
    (*filter)->onFootprintChanged();
  }
}

}  // namespace nav2_costmap_2d

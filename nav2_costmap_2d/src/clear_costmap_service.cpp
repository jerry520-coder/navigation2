// Copyright (c) 2018 Intel Corporation
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

#include <vector>
#include <string>
#include <algorithm>
#include <memory>

#include "nav2_costmap_2d/clear_costmap_service.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_costmap_2d
{

using std::vector;
using std::string;
using std::shared_ptr;
using std::any_of;
using ClearExceptRegion = nav2_msgs::srv::ClearCostmapExceptRegion; //清除代价图，reset_distance 指定的矩形区域除外
using ClearAroundRobot = nav2_msgs::srv::ClearCostmapAroundRobot; //清除一定距离内的成本计算图
using ClearEntirely = nav2_msgs::srv::ClearEntireCostmap; //清除成本图上的所有图层

ClearCostmapService::ClearCostmapService(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  Costmap2DROS & costmap)
: costmap_(costmap)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  reset_value_ = costmap_.getCostmap()->getDefaultValue();

  node->get_parameter("clearable_layers", clearable_layers_);// 获取参数"clearable_layers"，此参数指定了可清除的层

// 创建服务"clear_except_"，用于清除除指定区域外的成本图
  clear_except_service_ = node->create_service<ClearExceptRegion>(
    "clear_except_" + costmap_.getName(),
    std::bind(
      &ClearCostmapService::clearExceptRegionCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

 // 创建服务"clear_around_"，用于清除机器人周围的成本图
  clear_around_service_ = node->create_service<ClearAroundRobot>(
    "clear_around_" + costmap.getName(),
    std::bind(
      &ClearCostmapService::clearAroundRobotCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

 // 创建服务"clear_entirely_"，用于完全清除成本图
  clear_entire_service_ = node->create_service<ClearEntirely>(
    "clear_entirely_" + costmap_.getName(),
    std::bind(
      &ClearCostmapService::clearEntireCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void ClearCostmapService::clearExceptRegionCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearExceptRegion::Request> request,
  const shared_ptr<ClearExceptRegion::Response>/*response*/)
{
  RCLCPP_INFO(
    logger_, "%s",
    ("Received request to clear except a region the " + costmap_.getName()).c_str());

  clearRegion(request->reset_distance, true);// 调用clearRegion函数来处理请求，invert参数设置为true表示保留指定区域
}

void ClearCostmapService::clearAroundRobotCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearAroundRobot::Request> request,
  const shared_ptr<ClearAroundRobot::Response>/*response*/)
{
  clearRegion(request->reset_distance, false);
}

void ClearCostmapService::clearEntireCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ClearEntirely::Request>/*request*/,
  const std::shared_ptr<ClearEntirely::Response>/*response*/)
{
  RCLCPP_INFO(
    logger_, "%s",
    ("Received request to clear entirely the " + costmap_.getName()).c_str());

  clearEntirely(); // 调用clearEntirely函数来清除整个成本图
}


void ClearCostmapService::clearRegion(const double reset_distance, bool invert)
{
  double x, y;

// 获取机器人当前的位置
  if (!getPosition(x, y)) {
    RCLCPP_ERROR(
      logger_, "%s",
      "Cannot clear map because robot pose cannot be retrieved.");
    return;
  }


 // 获取所有可以被清除的层
  auto layers = costmap_.getLayeredCostmap()->getPlugins();

// 遍历所有层，只清除那些标记为可以清除的层
  for (auto & layer : *layers) {
    if (layer->isClearable()) {
      auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
      clearLayerRegion(costmap_layer, x, y, reset_distance, invert);
    }
  }

  // AlexeyMerzlyakov: No need to clear layer region for costmap filters
  // as they are always supposed to be not clearable.

  // AlexeyMerzlyakov：无需清除成本图过滤器的图层区域
  // 因为它们总是被认为是不可清除的。
}

void ClearCostmapService::clearLayerRegion(
  shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance,
  bool invert)
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap->getMutex()));

// 计算区域的边界
  double start_point_x = pose_x - reset_distance / 2;
  double start_point_y = pose_y - reset_distance / 2;
  double end_point_x = start_point_x + reset_distance;
  double end_point_y = start_point_y + reset_distance;

// 将世界坐标转换为地图坐标
  int start_x, start_y, end_x, end_y;
  costmap->worldToMapEnforceBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapEnforceBounds(end_point_x, end_point_y, end_x, end_y);

  // 清除或保留指定区域
  costmap->clearArea(start_x, start_y, end_x, end_y, invert);

 // 更新地图的边界，以便重新计算
  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
}

void ClearCostmapService::clearEntirely()
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getCostmap()->getMutex()));
  costmap_.resetLayers(); // 重置所有层
}

bool ClearCostmapService::getPosition(double & x, double & y) const
{
  geometry_msgs::msg::PoseStamped pose;
  if (!costmap_.getRobotPose(pose)) {
    return false;
  }

  x = pose.pose.position.x;
  y = pose.pose.position.y;

  return true;
}

}  // namespace nav2_costmap_2d

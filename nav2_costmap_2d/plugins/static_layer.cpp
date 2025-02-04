/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
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

#include "nav2_costmap_2d/static_layer.hpp"

#include <algorithm>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::StaticLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

StaticLayer::StaticLayer()
: map_buffer_(nullptr)
{
}

StaticLayer::~StaticLayer()
{
}

void
StaticLayer::onInitialize()
{
  global_frame_ = layered_costmap_->getGlobalFrameID();

  getParameters();

  rclcpp::QoS map_qos(10);  // initialize to default。默认队列长度为10
  if (map_subscribe_transient_local_) {
    map_qos.transient_local();// 设置QoS为瞬态局部模式
    map_qos.reliable();// 设置QoS为可靠模式
    map_qos.keep_last(1);// 仅保留最后一个消息
  }

  RCLCPP_INFO(
    logger_,
    "Subscribing to the map topic (%s) with %s durability",
    map_topic_.c_str(),
    map_subscribe_transient_local_ ? "transient local" : "volatile");

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&StaticLayer::incomingMap, this, std::placeholders::_1));

// 如果需要订阅地图更新
  if (subscribe_to_updates_) {
    RCLCPP_INFO(logger_, "Subscribing to updates");
    map_update_sub_ = node->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      map_topic_ + "_updates",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&StaticLayer::incomingUpdate, this, std::placeholders::_1));
  }
}

void
StaticLayer::activate()
{
}

void
StaticLayer::deactivate()
{
  dyn_params_handler_.reset();
}

void
StaticLayer::reset()
{
  has_updated_data_ = true;
  current_ = false;
}

void
StaticLayer::getParameters()
{
  int temp_lethal_threshold = 0;
  double temp_tf_tol = 0.0;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("subscribe_to_updates", rclcpp::ParameterValue(false));
  declareParameter("map_subscribe_transient_local", rclcpp::ParameterValue(true));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.0));
  declareParameter("map_topic", rclcpp::ParameterValue(""));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "subscribe_to_updates", subscribe_to_updates_);
  std::string private_map_topic, global_map_topic;
  node->get_parameter(name_ + "." + "map_topic", private_map_topic);
  node->get_parameter("map_topic", global_map_topic);
  if (!private_map_topic.empty()) {
    map_topic_ = private_map_topic;
  } else {
    map_topic_ = global_map_topic;
  }
  node->get_parameter(
    name_ + "." + "map_subscribe_transient_local",
    map_subscribe_transient_local_);
  node->get_parameter("track_unknown_space", track_unknown_space_);
  node->get_parameter("use_maximum", use_maximum_);
  node->get_parameter("lethal_cost_threshold", temp_lethal_threshold);
  node->get_parameter("unknown_cost_value", unknown_cost_value_);
  node->get_parameter("trinary_costmap", trinary_costmap_);
  node->get_parameter("transform_tolerance", temp_tf_tol);

  // Enforce bounds
  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  map_received_ = false;
  map_received_in_update_bounds_ = false;

  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &StaticLayer::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void
StaticLayer::processMap(const nav_msgs::msg::OccupancyGrid & new_map)
{
  RCLCPP_DEBUG(logger_, "StaticLayer: Process map");

  unsigned int size_x = new_map.info.width;
  unsigned int size_y = new_map.info.height;

 // 显示接收到的地图尺寸和分辨率
  RCLCPP_DEBUG(
    logger_,
    "StaticLayer: Received a %d X %d map at %f m/pix", size_x, size_y,
    new_map.info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D * master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
    master->getSizeInCellsY() != size_y ||
    master->getResolution() != new_map.info.resolution ||
    master->getOriginX() != new_map.info.origin.position.x ||
    master->getOriginY() != new_map.info.origin.position.y ||
    !layered_costmap_->isSizeLocked()))  // 如果地图的尺寸、分辨率或原点不匹配，调整代价地图的大小
  {
    // Update the size of the layered costmap (and all layers, including this one) // 更新层代价地图的大小（包括此层）
    RCLCPP_INFO(
      logger_,
      "StaticLayer: Resizing costmap to %d X %d at %f m/pix", size_x, size_y,
      new_map.info.resolution);

    layered_costmap_->resizeMap(
      size_x, size_y, new_map.info.resolution,
      new_map.info.origin.position.x,
      new_map.info.origin.position.y,
      true);

  } else if (size_x_ != size_x || size_y_ != size_y ||  // NOLINT
    resolution_ != new_map.info.resolution ||
    origin_x_ != new_map.info.origin.position.x ||
    origin_y_ != new_map.info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer // 仅更新此层代价地图的大小 
    RCLCPP_INFO(
      logger_,
      "StaticLayer: Resizing static layer to %d X %d at %f m/pix", size_x, size_y,
      new_map.info.resolution);

    resizeMap(
      size_x, size_y, new_map.info.resolution,
      new_map.info.origin.position.x, new_map.info.origin.position.y);
  }

  unsigned int index = 0;

  // we have a new map, update full size of map // 更新整张地图
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // initialize the costmap with static data // 用静态数据初始化代价地图
  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x; ++j) {
      unsigned char value = new_map.data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }

  map_frame_ = new_map.header.frame_id;

  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  has_updated_data_ = true; // 标记数据已更新

  current_ = true; // 标记地图当前有效
}

void
StaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  // 如果我们使用滚动成本图，则静态地图大小为与分层costmap的大小无关
  if (!layered_costmap_->isRolling()) {
    Costmap2D * master = layered_costmap_->getCostmap();
    resizeMap(
      master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
      master->getOriginX(), master->getOriginY());
  }
}

unsigned char
StaticLayer::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds // 检查静态值是否超过未知或致命阈值
  if (track_unknown_space_ && value == unknown_cost_value_) {
    return NO_INFORMATION;
  } else if (!track_unknown_space_ && value == unknown_cost_value_) {
    return FREE_SPACE;
  } else if (value >= lethal_threshold_) {
    return LETHAL_OBSTACLE;
  } else if (trinary_costmap_) {
    return FREE_SPACE;
  }

  double scale = static_cast<double>(value) / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void
StaticLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
{
  if (!map_received_) {
    processMap(*new_map);
    map_received_ = true;
    return;
  }
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  map_buffer_ = new_map;
}

void
StaticLayer::incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (update->y < static_cast<int32_t>(y_) ||
    y_ + height_ < update->y + update->height ||
    update->x < static_cast<int32_t>(x_) ||
    x_ + width_ < update->x + update->width)
  {
    // 更新超出静态层边界，打印警告信息并返回
    RCLCPP_WARN(
      logger_,
      "StaticLayer: Map update ignored. Exceeds bounds of static layer.\n"
      "Static layer origin: %d, %d   bounds: %d X %d\n"
      "Update origin: %d, %d   bounds: %d X %d",
      x_, y_, width_, height_, update->x, update->y, update->width,
      update->height);
    return;
  }

  if (update->header.frame_id != map_frame_) {
     // 更新的帧ID与当前地图帧ID不匹配，打印警告信息
    RCLCPP_WARN(
      logger_,
      "StaticLayer: Map update ignored. Current map is in frame %s "
      "but update was in frame %s",
      map_frame_.c_str(), update->header.frame_id.c_str());
  }

  // 处理接收到的地图更新数据，并更新代价地图。
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height; y++) {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width; x++) {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }

  has_updated_data_ = true;// 标记数据已更新
}


void
StaticLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  // 如果地图没有接收到，更新标志位并返回
  if (!map_received_) {
    map_received_in_update_bounds_ = false;
    return;
  }
  map_received_in_update_bounds_ = true;

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // If there is a new available map, load it. // 如果有新地图可用，加载它
  if (map_buffer_) {
    processMap(*map_buffer_);
    map_buffer_ = nullptr;
  }

 // 如果分层代价地图不是滚动的，并且没有更新数据或额外边界，则直接返回
  if (!layered_costmap_->isRolling() ) {
    if (!(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }

 // 使用额外的边界信息更新最小和最大边界
  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

   // 将地图坐标 (x_, y_) 转换为世界坐标，并更新最小边界
  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

// 将地图坐标 (x_ + width_, y_ + height_) 转换为世界坐标，并更新最大边界
  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false; // 更新数据处理完成，重置更新标志位
}

void
StaticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // 如果图层未启用，直接返回
  if (!enabled_) {
    return;
  }

  // 如果在更新边界时没有接收到地图，输出警告信息并返回
  if (!map_received_in_update_bounds_) {
    static int count = 0;
    // throttle warning down to only 1/10 message rate // 每10次输出一次警告信息
    if (++count == 10) {
      RCLCPP_WARN(logger_, "Can't update static costmap layer, no map received");
      count = 0;
    }
    return;
  }

  // 如果地图不是滚动的
  if (!layered_costmap_->isRolling()) {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer // 分层代价地图(master_grid)的坐标与此图层相同
    if (!use_maximum_) {
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j); // 使用真实覆盖更新代价
    } else {
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);// 使用最大值更新代价
    }
  } else {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer // 如果是滚动窗口，master_grid的坐标可能与此图层不同
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame // 可能甚至是在不同的坐标框架中
    geometry_msgs::msg::TransformStamped transform;
    try {
       // 查找从global_frame_到map_frame_的转换
      transform = tf_->lookupTransform(
        map_frame_, global_frame_, tf2::TimePointZero,
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "StaticLayer: %s", ex.what());
      return;
    }
    // Copy map data given proper transformations // 复制地图数据并进行适当的转换
    tf2::Transform tf2_transform;
    tf2::fromMsg(transform.transform, tf2_transform);

  // 遍历更新边界内的所有单元格
    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        // 将master_grid的坐标(i,j)转换为global_frame_的世界坐标(wx,wy)
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_ // 从global_frame_转换到map_frame_
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform * p;
        // Set master_grid with cell from map // 将地图中的单元格设置到master_grid中
        if (worldToMap(p.x(), p.y(), mx, my)) {
          if (!use_maximum_) {
            master_grid.setCost(i, j, getCost(mx, my));
          } else {
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
          }
        }
      }
    }
  }
  current_ = true;// 标记当前图层数据为最新
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult
StaticLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_name == name_ + "." + "map_subscribe_transient_local" ||
      param_name == name_ + "." + "map_topic" ||
      param_name == name_ + "." + "subscribe_to_updates")
    {
      RCLCPP_WARN(
        logger_, "%s is not a dynamic parameter "
        "cannot be changed while running. Rejecting parameter update.", param_name.c_str());
    } else if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "transform_tolerance") {
        transform_tolerance_ = tf2::durationFromSec(parameter.as_double());
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();

        x_ = y_ = 0;
        width_ = size_x_;
        height_ = size_y_;
        has_updated_data_ = true;
        current_ = false;
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_costmap_2d

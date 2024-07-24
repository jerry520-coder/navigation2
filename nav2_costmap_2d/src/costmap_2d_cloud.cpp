/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <vector>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_voxel_grid/voxel_grid.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"
#include "nav2_util/execution_timer.hpp"

static inline void mapToWorld3D(
  const unsigned int mx,
  const unsigned int my, const unsigned int mz,
  const double origin_x, const double origin_y, const double origin_z,
  const double x_resolution, const double y_resolution,
  const double z_resolution,
  double & wx, double & wy, double & wz)
{
  // returns the center point of the cell
  wx = origin_x + (mx + 0.5) * x_resolution;
  wy = origin_y + (my + 0.5) * y_resolution;
  wz = origin_z + (mz + 0.5) * z_resolution;
}

struct Cell
{
  double x;
  double y;
  double z;
  nav2_voxel_grid::VoxelStatus status;
};

typedef std::vector<Cell> V_Cell;

float g_colors_r[] = {0.0f, 0.0f, 1.0f};
float g_colors_g[] = {0.0f, 0.0f, 0.0f};
float g_colors_b[] = {0.0f, 1.0f, 0.0f};
float g_colors_a[] = {0.0f, 0.5f, 1.0f};

V_Cell g_marked;
V_Cell g_unknown;

rclcpp::Node::SharedPtr g_node;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_marked;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_unknown;

/**
 * @brief An helper function to fill pointcloud2 of both the marked and unknown points from voxel_grid
 * @param cloud PointCloud2 Ptr which needs to be filled
 * @param num_channels Represents the total number of points that are going to be filled
 * @param header Carries the header information that needs to be assigned to PointCloud2 header
 * @param g_cells contains the x, y, z values that needs to be added to the PointCloud2
 */

/**
 * @brief helper function，用于从体素网格填充the marked and unknown points的PointCloud2
 * @param cloud 需要填充的PointCloud2指针
 * @param num_channels 表示将要填充的点的总数
 * @param header 包含需要赋予PointCloud2头部的头信息
 * @param g_cells 包含需要添加到PointCloud2中的x、y、z值
 */
void pointCloud2Helper(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & cloud,
  uint32_t num_channels,
  std_msgs::msg::Header header,
  V_Cell & g_cells)
{
  // 设置点云的基本属性
  cloud->header = header;
  cloud->width = num_channels;// 点的数量
  cloud->height = 1;// 单行点云
  cloud->is_dense = true;// 稠密的
  cloud->is_bigendian = false;// 采用小端字节序
  sensor_msgs::PointCloud2Modifier modifier(*cloud); // 使用PointCloud2Modifier来方便地添加和设置字段
  modifier.setPointCloud2Fields(
    6, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "r", 1, sensor_msgs::msg::PointField::UINT8,
    "g", 1, sensor_msgs::msg::PointField::UINT8,
    "b", 1, sensor_msgs::msg::PointField::UINT8);

  // 创建迭代器，用于遍历和设置每个字段的数据
  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud, "b");


 // 遍历所有的Cell，填充点云数据
  for (uint32_t i = 0; i < num_channels; ++i) {
    Cell & c = g_cells[i];
    // assigning value to the point cloud2's iterator
    *iter_x = c.x;
    *iter_y = c.y;
    *iter_z = c.z;
    *iter_r = g_colors_r[c.status] * 255.0;// 设置红色分量
    *iter_g = g_colors_g[c.status] * 255.0;
    *iter_b = g_colors_b[c.status] * 255.0;

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }
}

void voxelCallback(const nav2_msgs::msg::VoxelGrid::ConstSharedPtr grid)
{
  // 检查接收的体素网格数据是否为空
  if (grid->data.empty()) {
    RCLCPP_ERROR(g_node->get_logger(), "Received empty voxel grid");
    return;
  }

 // 开始计时以跟踪操作的执行时间
  nav2_util::ExecutionTimer timer;
  timer.start();

  // 提取体素网格的元数据
  RCLCPP_DEBUG(g_node->get_logger(), "Received voxel grid");
  const std::string frame_id = grid->header.frame_id;
  const rclcpp::Time stamp = grid->header.stamp;
  const uint32_t * data = &grid->data.front();
  const double x_origin = grid->origin.x;
  const double y_origin = grid->origin.y;
  const double z_origin = grid->origin.z;
  const double x_res = grid->resolutions.x;
  const double y_res = grid->resolutions.y;
  const double z_res = grid->resolutions.z;
  const uint32_t x_size = grid->size_x;
  const uint32_t y_size = grid->size_y;
  const uint32_t z_size = grid->size_z;


 // 清空先前的数据存储器
  g_marked.clear();
  g_unknown.clear();
  uint32_t num_marked = 0;
  uint32_t num_unknown = 0;

  // 遍历体素数据，分别记录已知和未知状态的体素
  for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid) {
    for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid) {
      for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid) {
        nav2_voxel_grid::VoxelStatus status =
          nav2_voxel_grid::VoxelGrid::getVoxel(
          x_grid, y_grid,
          z_grid, x_size, y_size, z_size, data);
        if (status == nav2_voxel_grid::UNKNOWN) {
          Cell c;
          c.status = status;
          mapToWorld3D(
            x_grid, y_grid, z_grid, x_origin, y_origin,
            z_origin, x_res, y_res, z_res, c.x, c.y, c.z);

          g_unknown.push_back(c);

          ++num_unknown;
        } else if (status == nav2_voxel_grid::MARKED) {
          Cell c;
          c.status = status;
          mapToWorld3D(
            x_grid, y_grid, z_grid, x_origin, y_origin,
            z_origin, x_res, y_res, z_res, c.x, c.y, c.z);

          g_marked.push_back(c);

          ++num_marked;
        }
      }
    }
  }

  // 准备点云的头部信息 
  std_msgs::msg::Header pcl_header;
  pcl_header.frame_id = frame_id;
  pcl_header.stamp = stamp;

  // 创建点云消息，并使用helper函数填充，然后发布已知状态的点云
  {
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pointCloud2Helper(cloud, num_marked, pcl_header, g_marked);
    pub_marked->publish(std::move(cloud));
  }

  // 创建点云消息，并使用helper函数填充，然后发布未知状态的点云
  {
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pointCloud2Helper(cloud, num_unknown, pcl_header, g_unknown);
    pub_unknown->publish(std::move(cloud));
  }

 // 结束计时并记录发布点云的信息
  timer.end();
  RCLCPP_DEBUG(
    g_node->get_logger(), "Published %d points in %f seconds",
    num_marked + num_unknown, timer.elapsed_time_in_seconds());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("costmap_2d_cloud");

  RCLCPP_DEBUG(g_node->get_logger(), "Starting up costmap_2d_cloud");

  pub_marked = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "voxel_marked_cloud", 1);
  pub_unknown = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "voxel_unknown_cloud", 1);
  auto sub = g_node->create_subscription<nav2_msgs::msg::VoxelGrid>(
    "voxel_grid", rclcpp::SystemDefaultsQoS(), voxelCallback);

  rclcpp::spin(g_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}

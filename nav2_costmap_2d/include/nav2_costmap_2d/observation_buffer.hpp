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
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__OBSERVATION_BUFFER_HPP_
#define NAV2_COSTMAP_2D__OBSERVATION_BUFFER_HPP_

#include <vector>
#include <list>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/observation.hpp"
#include "nav2_util/lifecycle_node.hpp"


namespace nav2_costmap_2d
{
/**
 * @class ObservationBuffer
 * @brief Takes in point clouds from sensors, transforms them to the desired frame, and stores them
 */
class ObservationBuffer
{
public:
  /**
   * @brief  Constructs an observation buffer
   * @param  topic_name The topic of the observations, used as an identifier for error and warning messages
   * @param  observation_keep_time 以秒为单位定义观测数据的持久性，0 表示只保留最新的观测数据。Defines the persistence of observations in seconds, 0 means only keep the latest
   * @param  expected_update_rate 该缓冲区的更新频率，0 表示没有限制。How often this buffer is expected to be updated, 0 means there is no limit
   * @param  min_obstacle_height The minimum height of a hitpoint to be considered legal
   * @param  max_obstacle_height The minimum height of a hitpoint to be considered legal
   * @param  obstacle_max_range The range to which the sensor should be trusted for inserting obstacles
   * @param  obstacle_min_range The range from which the sensor should be trusted for inserting obstacles
   * @param  raytrace_max_range The range to which the sensor should be trusted for raytracing to clear out space
   * @param  raytrace_min_range The range from which the sensor should be trusted for raytracing to clear out space
   * @param  tf2_buffer A reference to a tf2 Buffer
   * @param  global_frame The frame to transform PointClouds into
   * @param  sensor_frame The frame of the origin of the sensor, can be left blank to be read from the messages
   * @param  tf_tolerance The amount of time to wait for a transform to be available when setting a new global frame
   */

  /**
 * @brief 构造一个观测数据缓冲区
 * @param topic_name 观测数据的主题，用作错误和警告消息的标识符
 * @param observation_keep_time 以秒为单位定义观测数据的持久性，0 表示只保留最新的观测数据
 * @param expected_update_rate 该缓冲区的预期更新频率，0 表示没有限制
 * @param min_obstacle_height 被视为合法的击中点的最小高度
 * @param max_obstacle_height 被视为合法的击中点的最大高度
 * @param obstacle_max_range 传感器用于插入障碍物的信任范围的最大距离
 * @param obstacle_min_range 传感器用于插入障碍物的信任范围的最小距离
 * @param raytrace_max_range 传感器用于光线追踪以清除空间的信任范围的最大距离
 * @param raytrace_min_range 传感器用于光线追踪以清除空间的信任范围的最小距离
 * @param tf2_buffer 对 tf2 缓冲区的引用
 * @param global_frame 将 PointCloud 转换为的坐标系
 * @param sensor_frame 传感器原点的坐标系，可以留空以从消息中读取
 * @param tf_tolerance 设置新的全局坐标系时等待变换可用的时间量
 */
  ObservationBuffer(
    const nav2_util::LifecycleNode::WeakPtr & parent,
    std::string topic_name,
    double observation_keep_time,
    double expected_update_rate,
    double min_obstacle_height, double max_obstacle_height, double obstacle_max_range,
    double obstacle_min_range,
    double raytrace_max_range, double raytrace_min_range, tf2_ros::Buffer & tf2_buffer,
    std::string global_frame,
    std::string sensor_frame,
    tf2::Duration tf_tolerance);

  /**
   * @brief  Destructor... cleans up
   */
  ~ObservationBuffer();

  /**
   * @brief  Transforms a PointCloud to the global frame and buffers it
   * <b>Note: The burden is on the user to make sure the transform is available... ie they should use a MessageNotifier</b>
   * @param  cloud The cloud to be buffered
   */

  /**
 * @brief 将 PointCloud 转换为全局坐标系并进行缓存
 * <b>注意：用户有责任确保变换是可用的...即，他们应该使用一个 MessageNotifier</b>
 * @param cloud 要进行缓存的点云数据
 */
  void bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud);

  /**
   * @brief  Pushes copies of all current observations onto the end of the vector passed in
   * @param  observations The vector to be filled
   */

  /**
 * @brief 将所有当前观测数据的副本推送到传入的向量的末尾
 * @param observations 要填充的向量
 */
  void getObservations(std::vector<Observation> & observations);

  /**
   * @brief  Check if the observation buffer is being update at its expected rate
   * @return True if it is being updated at the expected rate, false otherwise
   */
  bool isCurrent() const;

  /**
   * @brief  Lock the observation buffer
   */
  inline void lock()
  {
    lock_.lock();
  }

  /**
   * @brief  Lock the observation buffer
   */
  inline void unlock()
  {
    lock_.unlock();
  }

  /**
   * @brief Reset last updated timestamp
   */
  void resetLastUpdated();

private:
  /**
   * @brief  Removes any stale observations from the buffer list
   */
  void purgeStaleObservations();

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_costmap_2d")};
  tf2_ros::Buffer & tf2_buffer_;
  const rclcpp::Duration observation_keep_time_;
  const rclcpp::Duration expected_update_rate_;
  rclcpp::Time last_updated_;
  std::string global_frame_;
  std::string sensor_frame_;
  std::list<Observation> observation_list_;
  std::string topic_name_;
  double min_obstacle_height_, max_obstacle_height_;
  std::recursive_mutex lock_;  ///< @brief A lock for accessing data in callbacks safely
  double obstacle_max_range_, obstacle_min_range_, raytrace_max_range_, raytrace_min_range_;
  tf2::Duration tf_tolerance_;
};
}  // namespace nav2_costmap_2d
#endif  // NAV2_COSTMAP_2D__OBSERVATION_BUFFER_HPP_

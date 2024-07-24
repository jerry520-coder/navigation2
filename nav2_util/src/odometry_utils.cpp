// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#include <string>

#include "nav2_util/odometry_utils.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

namespace nav2_util
{

OdomSmoother::OdomSmoother(
  const rclcpp::Node::WeakPtr & parent,
  double filter_duration,
  const std::string & odom_topic)
: odom_history_duration_(rclcpp::Duration::from_seconds(filter_duration))
{
  auto node = parent.lock();
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&OdomSmoother::odomCallback, this, std::placeholders::_1));

  odom_cumulate_.twist.twist.linear.x = 0;
  odom_cumulate_.twist.twist.linear.y = 0;
  odom_cumulate_.twist.twist.linear.z = 0;
  odom_cumulate_.twist.twist.angular.x = 0;
  odom_cumulate_.twist.twist.angular.y = 0;
  odom_cumulate_.twist.twist.angular.z = 0;
}

OdomSmoother::OdomSmoother(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  double filter_duration,
  const std::string & odom_topic)
: odom_history_duration_(rclcpp::Duration::from_seconds(filter_duration))
{
  auto node = parent.lock();
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&OdomSmoother::odomCallback, this, std::placeholders::_1));

  odom_cumulate_.twist.twist.linear.x = 0;
  odom_cumulate_.twist.twist.linear.y = 0;
  odom_cumulate_.twist.twist.linear.z = 0;
  odom_cumulate_.twist.twist.angular.x = 0;
  odom_cumulate_.twist.twist.angular.y = 0;
  odom_cumulate_.twist.twist.angular.z = 0;
}

void OdomSmoother::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);

  // update cumulated odom only if history is not empty // 仅在历史记录不为空的情况下更新累积里程
  if (!odom_history_.empty()) {
    // to store current time // 存储当前时间
    auto current_time = rclcpp::Time(msg->header.stamp);

    // to store time of the first odom in history // 存储历史记录中最早的里程信息的时间
    auto front_time = rclcpp::Time(odom_history_.front().header.stamp);

    // update cumulated odom when duration has exceeded and pop earliest msg // 当时间间隔超过指定的历史记录时长时，更新累积里程并弹出最早的消息
    while (current_time - front_time > odom_history_duration_) {
      const auto & odom = odom_history_.front();   // 获取历史记录中最早的里程信息

       // 更新累积的线速度和角速度
      odom_cumulate_.twist.twist.linear.x -= odom.twist.twist.linear.x;
      odom_cumulate_.twist.twist.linear.y -= odom.twist.twist.linear.y;
      odom_cumulate_.twist.twist.linear.z -= odom.twist.twist.linear.z;
      odom_cumulate_.twist.twist.angular.x -= odom.twist.twist.angular.x;
      odom_cumulate_.twist.twist.angular.y -= odom.twist.twist.angular.y;
      odom_cumulate_.twist.twist.angular.z -= odom.twist.twist.angular.z;

      odom_history_.pop_front(); // 从历史记录中移除最早的里程信息

      // 如果历史记录为空，则跳出循环
      if (odom_history_.empty()) {
        break;
      }

      // update with the timestamp of earliest odom message in history // 更新最早的里程信息的时间戳
      front_time = rclcpp::Time(odom_history_.front().header.stamp);
    }
  }

  odom_history_.push_back(*msg); // 将当前里程信息添加到历史记录中
  updateState();
}

void OdomSmoother::updateState()
{
  const auto & odom = odom_history_.back(); // 获取历史记录中的最新里程信息

   // 更新累积的线速度和角速度
  odom_cumulate_.twist.twist.linear.x += odom.twist.twist.linear.x;
  odom_cumulate_.twist.twist.linear.y += odom.twist.twist.linear.y;
  odom_cumulate_.twist.twist.linear.z += odom.twist.twist.linear.z;
  odom_cumulate_.twist.twist.angular.x += odom.twist.twist.angular.x;
  odom_cumulate_.twist.twist.angular.y += odom.twist.twist.angular.y;
  odom_cumulate_.twist.twist.angular.z += odom.twist.twist.angular.z;

 // 将最新里程信息的头信息复制到 vel_smooth_
  vel_smooth_.header = odom.header;

    // 计算并更新平滑后的线速度和角速度
  vel_smooth_.twist.linear.x = odom_cumulate_.twist.twist.linear.x / odom_history_.size();
  vel_smooth_.twist.linear.y = odom_cumulate_.twist.twist.linear.y / odom_history_.size();
  vel_smooth_.twist.linear.z = odom_cumulate_.twist.twist.linear.z / odom_history_.size();
  vel_smooth_.twist.angular.x = odom_cumulate_.twist.twist.angular.x / odom_history_.size();
  vel_smooth_.twist.angular.y = odom_cumulate_.twist.twist.angular.y / odom_history_.size();
  vel_smooth_.twist.angular.z = odom_cumulate_.twist.twist.angular.z / odom_history_.size();
}

}  // namespace nav2_util

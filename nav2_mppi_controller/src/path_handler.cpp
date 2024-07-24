// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Dexory
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

#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace mppi
{

void PathHandler::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
  std::shared_ptr<tf2_ros::Buffer> buffer, ParametersHandler * param_handler)
{
  name_ = name;
  costmap_ = costmap;
  tf_buffer_ = buffer;
  auto node = parent.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(max_robot_pose_search_dist_, "max_robot_pose_search_dist", getMaxCostmapDist());
  getParam(prune_distance_, "prune_distance", 1.5);
  getParam(transform_tolerance_, "transform_tolerance", 0.1);
  getParam(enforce_path_inversion_, "enforce_path_inversion", false);
  if (enforce_path_inversion_) {
    getParam(inversion_xy_tolerance_, "inversion_xy_tolerance", 0.2);
    getParam(inversion_yaw_tolerance, "inversion_yaw_tolerance", 0.4);
    inversion_locale_ = 0u;
  }
}

std::pair<nav_msgs::msg::Path, PathIterator> PathHandler::getGlobalPlanConsideringBoundsInCostmapFrame(
  const geometry_msgs::msg::PoseStamped & global_pose)
{
  using nav2_util::geometry_utils::euclidean_distance;  // 使用命名空间中的欧几里得距离函数

  auto begin = global_plan_up_to_inversion_.poses.begin();

  // 将搜索最接近机器人的姿态的范围限制在路径上的 max_robot_pose_search_dist_ 范围内
  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_up_to_inversion_.poses.begin(), global_plan_up_to_inversion_.poses.end(),
    max_robot_pose_search_dist_);

  // 找到距离机器人最近的点
  auto closest_point = nav2_util::geometry_utils::min_by(
    begin, closest_pose_upper_bound,
    [&global_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(global_pose, ps);  // 返回两个姿态间的欧几里得距离
    });

  // 创建一个转换后的路径对象
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_->getGlobalFrameID();  // 设置路径的帧ID为 costmap 的全局帧ID
  transformed_plan.header.stamp = global_pose.header.stamp;  // 设置时间戳为传入全局姿态的时间戳

  // 找到路径中与机器人最近的点后，开始从该点开始遍历到路径末尾的迭代器
  auto pruned_plan_end =
    nav2_util::geometry_utils::first_after_integrated_distance(
    closest_point, global_plan_up_to_inversion_.poses.end(), prune_distance_);

  unsigned int mx, my;
  // 在 costmap 边界内找到最远的相关姿态
  // 同时在循环中将其转换到 costmap 帧
  for (auto global_plan_pose = closest_point; global_plan_pose != pruned_plan_end;
    ++global_plan_pose)
  {
    // 将全局路径帧中的姿态转换到 costmap 帧中
    geometry_msgs::msg::PoseStamped costmap_plan_pose;
    global_plan_pose->header.stamp = global_pose.header.stamp;  // 设置时间戳
    global_plan_pose->header.frame_id = global_plan_.header.frame_id;  // 设置帧ID
    transformPose(costmap_->getGlobalFrameID(), *global_plan_pose, costmap_plan_pose);  // 转换姿态到 costmap 帧

    // 检查姿态是否在 costmap 内
    if (!costmap_->getCostmap()->worldToMap(
        costmap_plan_pose.pose.position.x, costmap_plan_pose.pose.position.y, mx, my))
    {
      return {transformed_plan, closest_point};  // 如果不在 costmap 内，返回空路径和最近点的迭代器
    }

    // 将转换后的姿态填充到要返回的路径中
    transformed_plan.poses.push_back(costmap_plan_pose);
  }

  return {transformed_plan, closest_point};  // 返回转换后的路径和最近点的迭代器
}


geometry_msgs::msg::PoseStamped PathHandler::transformToGlobalPlanFrame(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_up_to_inversion_.poses.empty()) {
    throw std::runtime_error("Received plan with zero length");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_up_to_inversion_.header.frame_id, pose, robot_pose)) {
    throw std::runtime_error(
            "Unable to transform robot pose into global plan's frame");
  }

  return robot_pose;
}

nav_msgs::msg::Path PathHandler::transformPath(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // 找到与路径相关的边界
  // 将机器人的姿态转换到全局路径帧中
  geometry_msgs::msg::PoseStamped global_pose =
    transformToGlobalPlanFrame(robot_pose);

  // 获取考虑到在 Costmap 帧中的边界的全局路径
  auto [transformed_plan, lower_bound] = getGlobalPlanConsideringBoundsInCostmapFrame(global_pose);

  // 精简全局计划
  prunePlan(global_plan_up_to_inversion_, lower_bound);

  // 如果启用了路径反转并且反转位置不为 0
  if (enforce_path_inversion_ && inversion_locale_ != 0u) {
    // 如果在反转的容差范围内
    if (isWithinInversionTolerances(global_pose)) {
      // 对全局计划进行精简，去除反转位置之后的部分
      prunePlan(global_plan_, global_plan_.poses.begin() + inversion_locale_);
      global_plan_up_to_inversion_ = global_plan_;
      // 更新反转位置
      inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
    }
  }

  // 如果转换后的规划为空，则抛出运行时错误
  if (transformed_plan.poses.empty()) {
    throw std::runtime_error("Resulting plan has 0 poses in it.");
  }

  // 返回转换后的计划
  return transformed_plan;
}


bool PathHandler::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_buffer_->transform(
      in_pose, out_pose, frame,
      tf2::durationFromSec(transform_tolerance_));
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double PathHandler::getMaxCostmapDist()
{
  const auto & costmap = costmap_->getCostmap();
  return static_cast<double>(std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY())) *
         costmap->getResolution() * 0.50;
}

void PathHandler::setPath(const nav_msgs::msg::Path & plan)
{
  // 将传入的路径规划消息赋值给全局路径
  global_plan_ = plan;

  // 复制一份全局路径到另一个变量
  global_plan_up_to_inversion_ = global_plan_;

  // 如果设置了强制路径反转
  if (enforce_path_inversion_) {
    // 调用函数移除第一次反转后的路径点
    inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
  }
}


nav_msgs::msg::Path & PathHandler::getPath() {return global_plan_;}

void PathHandler::prunePlan(nav_msgs::msg::Path & plan, const PathIterator end)
{
  plan.poses.erase(plan.poses.begin(), end);
}

bool PathHandler::isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose) 
{
// Keep full path if we are within tolerance of the inversion pose
  const auto last_pose = global_plan_up_to_inversion_.poses.back();
  
  // 计算机器人当前位姿与反转路径上最后一个位姿的直线距离
  float distance = hypotf(
    robot_pose.pose.position.x - last_pose.pose.position.x,  // x轴上的差值
    robot_pose.pose.position.y - last_pose.pose.position.y);  // y轴上的差值

  // 计算机器人当前航向与反转路径上最后一个位姿航向之间的最短角度差
  float angle_distance = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation),  // 当前值姿的航向
    tf2::getYaw(last_pose.pose.orientation));    // 反转路径上最后一个位姿的航向

  // 判断距离和角度差是否都在设定的容忍范围内
  // 如果都在容忍范围内，则认为机器人在反转容忍内
  return distance <= inversion_xy_tolerance_ && fabs(angle_distance) <= inversion_yaw_tolerance;
}

}  // namespace mppi

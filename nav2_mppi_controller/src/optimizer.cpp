// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/optimizer.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace mppi
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void Optimizer::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  ParametersHandler * param_handler)
{
  parent_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  parameters_handler_ = param_handler;

  auto node = parent_.lock(); //lock()函数是weak_ptr的成员函数，用于尝试将weak_ptr转换为shared_ptr。如果weak_ptr指向的对象仍然存在，则lock()会返回一个指向该对象的shared_ptr；如果对象已经被销毁了，lock()则会返回一个空的shared_ptr。
  logger_ = node->get_logger(); // 获取父节点的日志记录器

  getParams();

  critic_manager_.on_configure(parent_, name_, costmap_ros_, parameters_handler_);
  noise_generator_.initialize(settings_, isHolonomic(), name_, parameters_handler_);// 初始化噪声生成器

  reset();// 重置优化器的状态
}

void Optimizer::shutdown()
{
  noise_generator_.shutdown();
}

void Optimizer::getParams()
{
  std::string motion_model_name; // 运动模型名称

  auto & s = settings_; // 引用优化器的设置
  auto getParam = parameters_handler_->getParamGetter(name_); // 获取参数的函数，传入参数名
  auto getParentParam = parameters_handler_->getParamGetter(""); // 获取父节点参数的函数，不传入参数名，表示使用默认参数

  // 获取各个参数，如果参数不存在，则使用默认值
  getParam(s.model_dt, "model_dt", 0.05f); // 模型时间间隔
  getParam(s.time_steps, "time_steps", 56); // 时间步数
  getParam(s.batch_size, "batch_size", 1000); // 批大小
  getParam(s.iteration_count, "iteration_count", 1); // 迭代次数
  getParam(s.temperature, "temperature", 0.3f); // 温度
  getParam(s.gamma, "gamma", 0.015f); // Gamma参数
  getParam(s.base_constraints.vx_max, "vx_max", 0.5); // 最大线速度
  getParam(s.base_constraints.vx_min, "vx_min", -0.35); // 最小线速度
  getParam(s.base_constraints.vy, "vy_max", 0.5); // Y轴最大速度
  getParam(s.base_constraints.wz, "wz_max", 1.9); // 最大角速度
  getParam(s.sampling_std.vx, "vx_std", 0.2); // 线速度采样标准差
  getParam(s.sampling_std.vy, "vy_std", 0.2); // Y轴速度采样标准差
  getParam(s.sampling_std.wz, "wz_std", 0.4); // 角速度采样标准差
  getParam(s.retry_attempt_limit, "retry_attempt_limit", 1); // 重试尝试次数限制

  getParam(motion_model_name, "motion_model", std::string("DiffDrive")); // 获取运动模型名称，如果不存在则使用默认值"DiffDrive"

  s.constraints = s.base_constraints; // 设置约束为基础约束
  setMotionModel(motion_model_name); // 设置运动模型
  parameters_handler_->addPostCallback([this]() {reset();}); // 添加回调函数，在参数更新后调用reset函数重置优化器状态

  double controller_frequency; // 控制器频率
  getParentParam(controller_frequency, "controller_frequency", 0.0, ParameterType::Static); // 获取父节点的控制器频率参数，如果不存在则使用默认值0.0
  setOffset(controller_frequency); // 设置偏移量
}


void Optimizer::setOffset(double controller_frequency)
{
  // 计算控制器周期，即每次控制之间的时间间隔
  const double controller_period = 1.0 / controller_frequency;
  
  // 设置一个非常小的正数，用于浮点数比较时避免精度问题
  constexpr double eps = 1e-6;

  // 如果控制器周期加上一个极小值仍然小于模型的时间步长
  if ((controller_period + eps) < settings_.model_dt) {
    // 在日志中发出警告，建议将控制器周期设置为与模型的时间步长相等
    RCLCPP_WARN(
      logger_,
      "Controller period is less then model dt, consider setting it equal");
  }
  // 如果控制器周期与模型的时间步长相差极小（可认为相等）
  else if (abs(controller_period - settings_.model_dt) < eps) {
    // 在日志中记录信息，表示控制序列的偏移（shifting）功能将被启用
    RCLCPP_INFO(
      logger_,
      "Controller period is equal to model dt. Control sequence shifting is ON");
    // 启用控制序列的偏移功能
    settings_.shift_control_sequence = true;
  }
  // 如果控制器周期大于模型的时间步长
  else {
    // 抛出异常，提示用户将控制器周期设置为与模型的时间步长相等
    throw std::runtime_error(
            "Controller period more then model dt, set it equal to model dt");
  }
}

void Optimizer::reset()
{
  // 重置状态
  state_.reset(settings_.batch_size, settings_.time_steps);
  
  // 重置控制序列
  control_sequence_.reset(settings_.time_steps);
  
  // 重置控制历史记录
  control_history_[0] = {0.0, 0.0, 0.0};
  control_history_[1] = {0.0, 0.0, 0.0};
  control_history_[2] = {0.0, 0.0, 0.0};
  control_history_[3] = {0.0, 0.0, 0.0};
  
  // 重置成本
  costs_ = xt::zeros<float>({settings_.batch_size});
  
  // 重置生成的轨迹
  generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);
  
  // 重置噪声生成器
  noise_generator_.reset(settings_, isHolonomic());
  
  // 输出日志信息
  RCLCPP_INFO(logger_, "Optimizer reset");
}


geometry_msgs::msg::TwistStamped Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  prepare(robot_pose, robot_speed, plan, goal_checker); // 准备数据

  // 进行优化直到没有失败的标志出现
  do {
    optimize(); // 进行一次优化
  } while (fallback(critics_data_.fail_flag)); // 如果存在失败标志则继续优化

  // 对控制序列进行Savitsky-Golay滤波
  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);

  // 从控制序列中获取控制指令
  auto control = getControlFromSequenceAsTwist(plan.header.stamp);

  // 如果设置了控制序列的移位，进行移位操作
  if (settings_.shift_control_sequence) {
    shiftControlSequence();
  }

  return control; // 返回控制指令
}

void Optimizer::optimize()
{
  // 进行指定次数的优化迭代
  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generateNoisedTrajectories(); // 生成加噪轨迹
    critic_manager_.evalTrajectoriesScores(critics_data_); // 评估轨迹分数
    updateControlSequence(); // 更新控制序列
  }
}


bool Optimizer::fallback(bool fail)
{
  // 当优化计算失败时进行回退

  // 定义一个静态变量用来记录回退次数
  static size_t counter = 0;

  // 如果计算成功，则将计数器重置为0，并返回false
  if (!fail) {
    counter = 0;
    return false;
  }

  // 如果计算失败，则调用reset方法重置优化器状态
  reset();

  // 增加计数器值
  if (++counter > settings_.retry_attempt_limit) {
    // 如果超过重试次数限制，则将计数器重置为0，并抛出运行时异常
    counter = 0;
    throw std::runtime_error("Optimizer fail to compute path");
  }

  // 如果未超过重试次数限制，则返回true，表示可以进行重试
  return true;
}


void Optimizer::prepare(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  state_.pose = robot_pose; // 设置优化器状态的机器人姿态
  state_.speed = robot_speed; // 设置优化器状态的机器人速度
  path_ = utils::toTensor(plan); // 将路径转换为Tensor对象

  // 将代价初始化为0
  costs_.fill(0);

  // 初始化临界值评估器的数据
  critics_data_.fail_flag = false; // 失败标志重置为false
  critics_data_.goal_checker = goal_checker; // 设置目标检测器
  critics_data_.motion_model = motion_model_; // 设置运动模型
  critics_data_.furthest_reached_path_point.reset(); // 重置最远到达路径点
  critics_data_.path_pts_valid.reset(); // 重置路径点有效性
}


void Optimizer::shiftControlSequence()
{
  using namespace xt::placeholders;  // NOLINT

  // 将控制序列中的线速度 vx 和角速度 wz 向前移动一个位置
  control_sequence_.vx = xt::roll(control_sequence_.vx, -1); //xt::roll(array, n) 将数组 array 的内容向前（负数 n）或向后（正数 n）循环移动 n 个位置。
  control_sequence_.wz = xt::roll(control_sequence_.wz, -1);

  // 将倒数第二个元素复制到最后一个位置
  xt::view(control_sequence_.vx, -1) =
    xt::view(control_sequence_.vx, -2); //-1 和 -2 是负索引，分别表示数组的最后一个元素和倒数第二个元素。这种操作可以确保序列的最后两个元素是相同的，可能是为了避免在使用滤波器时出现边界效应
  xt::view(control_sequence_.wz, -1) =
    xt::view(control_sequence_.wz, -2);

  // 如果是全向移动车辆
  if (isHolonomic()) {
    // 将控制序列中的侧向速度 vy 向前移动一个位置
    control_sequence_.vy = xt::roll(control_sequence_.vy, -1);
    // 将倒数第二个元素复制到最后一个位置
    xt::view(control_sequence_.vy, -1) =
      xt::view(control_sequence_.vy, -2);
  }
}



void Optimizer::generateNoisedTrajectories()// 生成带有噪声的轨迹
{
  // 使用噪声生成器设置带有噪声的控制参数
  noise_generator_.setNoisedControls(state_, control_sequence_);
  
  // 生成下一步的噪声
  noise_generator_.generateNextNoises();
  
  // 更新状态的速度信息
  updateStateVelocities(state_);
  
  // 根据状态的速度信息对生成的轨迹进行积分
  integrateStateVelocities(generated_trajectories_, state_);
}

bool Optimizer::isHolonomic() const {return motion_model_->isHolonomic();}

void Optimizer::applyControlSequenceConstraints()
{
  auto & s = settings_;  // 获取设置引用

  // 如果是全向机器人，对 vy 进行约束裁剪
  if (isHolonomic()) {
    control_sequence_.vy = xt::clip(control_sequence_.vy, -s.constraints.vy, s.constraints.vy);
  }

  // 对 vx 进行约束裁剪
  control_sequence_.vx = xt::clip(control_sequence_.vx, s.constraints.vx_min, s.constraints.vx_max);

  // 对 wz 进行约束裁剪
  control_sequence_.wz = xt::clip(control_sequence_.wz, -s.constraints.wz, s.constraints.wz);

  // 调用运动模型的 applyConstraints 方法，应用额外的约束
  motion_model_->applyConstraints(control_sequence_);
}


void Optimizer::updateStateVelocities(
  models::State & state) const
{
  updateInitialStateVelocities(state);
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(
  models::State & state) const  // 需要更新的状态对象
{

  //通过 xt::view 获取数组的特定部分的视图，而不是整个数组的副本。这样可以节省内存，尤其是当处理大型数组时
  //不同于直接创建副本或子数组，使用视图可以避免复制数据

  // 更新 x 方向上的初始速度信息
  xt::noalias(xt::view(state.vx, xt::all(), 0)) = state.speed.linear.x; //表示获取 state.vx 的所有行（xt::all()）和第一列（0 列）的视图，即更新 x 方向上的初始速度信息。

  // 更新 z 轴上的初始速度信息
  xt::noalias(xt::view(state.wz, xt::all(), 0)) = state.speed.angular.z;

  // 如果是全向机器人，更新 y 方向上的初始速度信息
  if (isHolonomic()) {
    xt::noalias(xt::view(state.vy, xt::all(), 0)) = state.speed.linear.y;
  }
}


void Optimizer::propagateStateVelocitiesFromInitials(
  models::State & state) const
{
  motion_model_->predict(state);
}

void Optimizer::integrateStateVelocities(
  xt::xtensor<float, 2> & trajectory,
  const xt::xtensor<float, 2> & sequence) const
{
  // 获取初始偏航角
  float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);

  // 从速度序列中提取 vx、vy、wz（如果是全向移动机器人）
  const auto vx = xt::view(sequence, xt::all(), 0);
  const auto vy = xt::view(sequence, xt::all(), 2);
  const auto wz = xt::view(sequence, xt::all(), 1);

  // 提取轨迹的 x、y、yaw（偏航角） 列
  auto traj_x = xt::view(trajectory, xt::all(), 0);
  auto traj_y = xt::view(trajectory, xt::all(), 1);
  auto traj_yaws = xt::view(trajectory, xt::all(), 2);

  // 计算偏航角的变化，并累积到轨迹的偏航角列中
  xt::noalias(traj_yaws) = xt::cumsum(wz * settings_.model_dt, 0) + initial_yaw;

  // 创建用于存储 cos(yaw) 和 sin(yaw) 的数组
  auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

  // 提取偏航角列的偏移后部分
  const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

  // 计算初始时刻的 cos(yaw) 和 sin(yaw)，并对后续部分进行计算
  xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
  xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

  // 计算每个时间步的 x 和 y 分量的增量
  auto && dx = xt::eval(vx * yaw_cos);
  auto && dy = xt::eval(vx * yaw_sin);

  // 如果是全向移动机器人，考虑侧向速度对 x 和 y 的影响
  if (isHolonomic()) {
    dx = dx - vy * yaw_sin;
    dy = dy + vy * yaw_cos;
  }

  // 根据增量累积计算位置轨迹的 x 和 y 列
  xt::noalias(traj_x) = state_.pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 0);
  xt::noalias(traj_y) = state_.pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 0);
}


void Optimizer::integrateStateVelocities(
  models::Trajectories & trajectories,
  const models::State & state) const
{
  // 获取初始偏航角
  const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

  // 计算偏航角的累积和
  xt::noalias(trajectories.yaws) =
    xt::cumsum(state.wz * settings_.model_dt, 1) + initial_yaw; //array：要进行累积和计算的数组。axis：可选参数，指定沿着哪个轴进行累积和操作。默认值为 0，表示沿着第一个轴（行）进行累积和。1,沿着列的方向计算累积和

  // 截取偏航角数组的第一列，作为后续计算用
  const auto yaws_cutted = xt::view(trajectories.yaws, xt::all(), xt::range(0, -1));

  // 计算偏航角对应的余弦和正弦值
  auto && yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape()); //根据 trajectories.yaws 的形状创建一个相同形状的新的 xtensor 对象，并且数据类型为 float。这个新的 xtensor 对象用于存储后续的计算结果
  auto && yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) = xt::cos(yaws_cutted);
  xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) = xt::sin(yaws_cutted);

  // 计算车辆的 x 和 y 方向上的位移增量
  auto && dx = xt::eval(state.vx * yaw_cos); //xt::xtensor 的计算中，有时候一些表达式并不会立即计算，而是在需要时才计算，这种延迟计算可以提高性能和内存使用效率。但是有时候需要立即计算，就可以使用 xt::eval
  auto && dy = xt::eval(state.vx * yaw_sin);

  // 如果是全向机器人，需要考虑 y 方向上的速度
  if (isHolonomic()) {
    dx = dx - state.vy * yaw_sin;
    dy = dy + state.vy * yaw_cos;
  }

  // 根据速度信息计算车辆的 x 和 y 方向上的位置
  xt::noalias(trajectories.x) = state.pose.pose.position.x +
    xt::cumsum(dx * settings_.model_dt, 1);
  xt::noalias(trajectories.y) = state.pose.pose.position.y +
    xt::cumsum(dy * settings_.model_dt, 1);
}


xt::xtensor<float, 2> Optimizer::getOptimizedTrajectory()
{
  // 创建最优轨迹的容器
  auto && sequence =
    xt::xtensor<float, 2>::from_shape({settings_.time_steps, isHolonomic() ? 3u : 2u}); //auto && sequence 的写法结合了自动类型推导和右值引用
  auto && trajectories = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3}); //创建一个形状为 (settings_.time_steps, 3) 的二维数组，元素类型为 float

  // 将控制序列中的速度和角速度放入对应的列
  xt::noalias(xt::view(sequence, xt::all(), 0)) = control_sequence_.vx;
  xt::noalias(xt::view(sequence, xt::all(), 1)) = control_sequence_.wz;

  // 如果是全向移动机器人，将控制序列中的侧向速度也放入第三列
  if (isHolonomic()) {
    xt::noalias(xt::view(sequence, xt::all(), 2)) = control_sequence_.vy;
  }

  // 对速度进行积分得到位置轨迹
  integrateStateVelocities(trajectories, sequence);

  // 返回最优轨迹
  return std::move(trajectories);
}

void Optimizer::updateControlSequence()
{
  // 获取设置的引用
  auto & s = settings_;
  // 计算速度 vx 和 wz 的有界噪声
  auto bounded_noises_vx = state_.cvx - control_sequence_.vx; //将 control_sequence_.vx 这个一维数组的每个元素都从 state_.cvx 的对应列中减去
  auto bounded_noises_wz = state_.cwz - control_sequence_.wz;
  // 更新成本，考虑 vx 的有界噪声
  // 将这两个数组对应位置的元素相乘
  xt::noalias(costs_) +=
    s.gamma / powf(s.sampling_std.vx, 2) * xt::sum(
    xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);//immediate 是一个参数，它在这里的作用是指示要立即执行 amin 函数，而不是延迟执行。
  // 更新成本，考虑 wz 的有界噪声
  xt::noalias(costs_) +=
    s.gamma / powf(s.sampling_std.wz, 2) * xt::sum(
    xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

  // 如果是全向移动的话
  if (isHolonomic()) {
    // 计算 vy 的有界噪声
    auto bounded_noises_vy = state_.cvy - control_sequence_.vy;
    // 更新成本，考虑 vy 的有界噪声
    xt::noalias(costs_) +=
      s.gamma / powf(s.sampling_std.vy, 2) * xt::sum(
      xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy,
      1, immediate);
  }

  // 成本归一化
  auto && costs_normalized = costs_ - xt::amin(costs_, immediate); //将 costs_ 中的每个元素减去 costs_ 中的最小值。这样做的目的是将 costs_ 的范围移动到非负数范围内
  // 计算指数
  auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
  // 计算 softmax
  auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
  // 扩展 softmax
  auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));//// 将一维数组 softmaxes 扩展为二维数组，每个元素变成一个列向量

  // 更新控制序列 vx
  xt::noalias(control_sequence_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
  // 更新控制序列 wz
  xt::noalias(control_sequence_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
  // 如果是全向移动的话，更新控制序列 vy
  if (isHolonomic()) {
    xt::noalias(control_sequence_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
  }

  // 应用控制序列约束
  applyControlSequenceConstraints();
}

geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
  const builtin_interfaces::msg::Time & stamp)
{
  // 如果设置为偏移控制序列，则偏移量为1，否则为0
  unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

  // 从控制序列中获取线速度 vx 和角速度 wz
  auto vx = control_sequence_.vx(offset);
  auto wz = control_sequence_.wz(offset);

  // 如果是全向移动车辆
  if (isHolonomic()) {
    // 获取侧向速度 vy
    auto vy = control_sequence_.vy(offset);
    // 调用工具函数将 vx、vy、wz 转换为 TwistStamped 消息
    return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
  }

  // 如果不是全向移动车辆，则只有线速度 vx 和角速度 wz
  // 调用工具函数将 vx、wz 转换为 TwistStamped 消息
  return utils::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());
}


void Optimizer::setMotionModel(const std::string & model)
{
  if (model == "DiffDrive") {
    motion_model_ = std::make_shared<DiffDriveMotionModel>();
  } else if (model == "Omni") {
    motion_model_ = std::make_shared<OmniMotionModel>();
  } else if (model == "Ackermann") {
    motion_model_ = std::make_shared<AckermannMotionModel>(parameters_handler_);
  } else {
    throw std::runtime_error(
            std::string(
              "Model " + model + " is not valid! Valid options are DiffDrive, Omni, "
              "or Ackermann"));
  }
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage) 
{
  // 引用设置对象
  auto & s = settings_;

  // 如果速度限制等于NO_SPEED_LIMIT（通常是一个特定的常量，表示没有速度限制），
  // 则将约束条件重置为基本约束
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    s.constraints.vx_max = s.base_constraints.vx_max; // 最大线速度
    s.constraints.vx_min = s.base_constraints.vx_min; // 最小线速度
    s.constraints.vy = s.base_constraints.vy; // 垂直线速度
    s.constraints.wz = s.base_constraints.wz; // 角速度
  }
  // 如果speed_limit不是NO_SPEED_LIMIT，则根据percentage参数设置速度限制
  else {
    // 如果percentage为true，表示速度限制是以百分比形式给出的
    if (percentage) {
      // 计算比例因子，即速度限制除以100
      double ratio = speed_limit / 100.0;
      // 将所有速度约束乘以比例因子，以应用百分比限制
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
    // 如果percentage为false，表示速度限制是以绝对值形式给出的
    else {
      // 计算比例因子，即速度限制除以当前的最大线速度
      double ratio = speed_limit / s.base_constraints.vx_max;
      // 将所有速度约束乘以比例因子，以应用绝对值速度限制
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
  }
}

models::Trajectories & Optimizer::getGeneratedTrajectories()
{
  return generated_trajectories_;
}

}  // namespace mppi

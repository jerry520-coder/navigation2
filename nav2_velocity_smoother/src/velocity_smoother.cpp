// Copyright (c) 2022 Samsung Research
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

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_velocity_smoother/velocity_smoother.hpp"

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;
using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace nav2_velocity_smoother
{

VelocitySmoother::VelocitySmoother(const rclcpp::NodeOptions & options)
: LifecycleNode("velocity_smoother", "", options),
  last_command_time_{0, 0, get_clock()->get_clock_type()}
{
}

VelocitySmoother::~VelocitySmoother()
{
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
}

nav2_util::CallbackReturn
VelocitySmoother::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring velocity smoother");
  auto node = shared_from_this();
  std::string feedback_type;
  double velocity_timeout_dbl;

  // Smoothing metadata
  declare_parameter_if_not_declared(node, "smoothing_frequency", rclcpp::ParameterValue(20.0));
  declare_parameter_if_not_declared(
    node, "feedback", rclcpp::ParameterValue(std::string("OPEN_LOOP")));
  declare_parameter_if_not_declared(node, "scale_velocities", rclcpp::ParameterValue(false));
  node->get_parameter("smoothing_frequency", smoothing_frequency_);
  node->get_parameter("feedback", feedback_type);
  node->get_parameter("scale_velocities", scale_velocities_);

  // Kinematics
  declare_parameter_if_not_declared(
    node, "max_velocity", rclcpp::ParameterValue(std::vector<double>{0.50, 0.0, 2.5}));
  declare_parameter_if_not_declared(
    node, "min_velocity", rclcpp::ParameterValue(std::vector<double>{-0.50, 0.0, -2.5}));
  declare_parameter_if_not_declared(
    node, "max_accel", rclcpp::ParameterValue(std::vector<double>{2.5, 0.0, 3.2}));
  declare_parameter_if_not_declared(
    node, "max_decel", rclcpp::ParameterValue(std::vector<double>{-2.5, 0.0, -3.2}));
  node->get_parameter("max_velocity", max_velocities_);
  node->get_parameter("min_velocity", min_velocities_);
  node->get_parameter("max_accel", max_accels_);
  node->get_parameter("max_decel", max_decels_);

  for (unsigned int i = 0; i != 3; i++) {
    if (max_decels_[i] > 0.0) {
      throw std::runtime_error(
              "Positive values set of deceleration! These should be negative to slow down!");
    }
    if (max_accels_[i] < 0.0) {
      throw std::runtime_error(
              "Negative values set of acceleration! These should be positive to speed up!");
    }
    if (min_velocities_[i] > max_velocities_[i]) {
      throw std::runtime_error(
              "Min velocities are higher than max velocities!");
    }
  }

  // Get feature parameters
  declare_parameter_if_not_declared(node, "odom_topic", rclcpp::ParameterValue("odom"));
  declare_parameter_if_not_declared(node, "odom_duration", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, "deadband_velocity", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
  declare_parameter_if_not_declared(node, "velocity_timeout", rclcpp::ParameterValue(1.0));
  node->get_parameter("odom_topic", odom_topic_);
  node->get_parameter("odom_duration", odom_duration_);
  node->get_parameter("deadband_velocity", deadband_velocities_);
  node->get_parameter("velocity_timeout", velocity_timeout_dbl); //dbl 是一个缩写，表示 “double”
  velocity_timeout_ = rclcpp::Duration::from_seconds(velocity_timeout_dbl);

  if (max_velocities_.size() != 3 || min_velocities_.size() != 3 ||
    max_accels_.size() != 3 || max_decels_.size() != 3 || deadband_velocities_.size() != 3)
  {
    throw std::runtime_error(
            "Invalid setting of kinematic and/or deadband limits!"
            " All limits must be size of 3 representing (x, y, theta).");
  }

  // Get control type
  if (feedback_type == "OPEN_LOOP") {
    open_loop_ = true;
  } else if (feedback_type == "CLOSED_LOOP") {
    open_loop_ = false;
    odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(node, odom_duration_, odom_topic_);
  } else {
    throw std::runtime_error("Invalid feedback_type, options are OPEN_LOOP and CLOSED_LOOP.");
  }

  // Setup inputs / outputs
  smoothed_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 1);
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(1),
    std::bind(&VelocitySmoother::inputCommandCallback, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VelocitySmoother::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  smoothed_cmd_pub_->on_activate();
  double timer_duration_ms = 1000.0 / smoothing_frequency_;
  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(timer_duration_ms)),
    std::bind(&VelocitySmoother::smootherTimer, this));

  dyn_params_handler_ = this->add_on_set_parameters_callback(
    std::bind(&VelocitySmoother::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VelocitySmoother::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  smoothed_cmd_pub_->on_deactivate();
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VelocitySmoother::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  smoothed_cmd_pub_.reset();
  odom_smoother_.reset();
  cmd_sub_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VelocitySmoother::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void VelocitySmoother::inputCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // If message contains NaN or Inf, ignore  // 如果消息包含 NaN 或 Inf，忽略该消息
  if (!nav2_util::validateTwist(*msg)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }

  command_ = msg; // 更新命令消息
  last_command_time_ = now();  // 记录最后一次接收命令的时间
}

double VelocitySmoother::findEtaConstraint(
  const double v_curr, const double v_cmd, const double accel, const double decel)
{
  // Exploiting vector scaling properties // 利用向量缩放性质
  double dv = v_cmd - v_curr;

  double v_component_max; // 最大速度分量
  double v_component_min;

  // Accelerating if magnitude of v_cmd is above magnitude of v_curr
  // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
  // Decelerating otherwise
  // 如果指令速度的大小大于当前速度的大小，并且指令速度和当前速度的符号相同（即速度不会通过0.0），则加速
  // 否则减速
  if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
    // 计算加速度下的最大和最小速度分量
    v_component_max = accel / smoothing_frequency_;
    v_component_min = -accel / smoothing_frequency_;
  } else {
    // 计算减速度下的最大和最小速度分量
    v_component_max = -decel / smoothing_frequency_;
    v_component_min = decel / smoothing_frequency_;
  }

// 如果速度差大于最大速度分量，则返回缩放因子
  if (dv > v_component_max) {
    return v_component_max / dv;
  }

 // 如果速度差小于最小速度分量，则返回缩放因子
  if (dv < v_component_min) {
    return v_component_min / dv;
  }

  return -1.0; // 否则返回 -1.0 表示无需缩放
}

double VelocitySmoother::applyConstraints(
  const double v_curr, const double v_cmd,
  const double accel, const double decel, const double eta)
{
  double dv = v_cmd - v_curr;

  double v_component_max; // 最大速度分量
  double v_component_min;

  // Accelerating if magnitude of v_cmd is above magnitude of v_curr
  // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
  // Decelerating otherwise

  // 如果指令速度的大小大于当前速度的大小，并且指令速度和当前速度的符号相同（即速度不会通过0.0），则加速
  // 否则减速
  if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
    v_component_max = accel / smoothing_frequency_;
    v_component_min = -accel / smoothing_frequency_;
  } else {
    v_component_max = -decel / smoothing_frequency_;
    v_component_min = decel / smoothing_frequency_;
  }

  return v_curr + std::clamp(eta * dv, v_component_min, v_component_max);
}

void VelocitySmoother::smootherTimer()
{
  // Wait until the first command is received // 等待直到接收到第一个命令
  if (!command_) {
    return;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

  // Check for velocity timeout. If nothing received, publish zeros to apply deceleration // 检查速度命令是否超时。如果未接收到新命令，发布零速度以应用减速度
  if (now() - last_command_time_ > velocity_timeout_) {
    if (last_cmd_ == geometry_msgs::msg::Twist() || stopped_) {
      stopped_ = true;
      return;
    }
    *command_ = geometry_msgs::msg::Twist();
  }

  stopped_ = false;

  // Get current velocity based on feedback type // 根据反馈类型获取当前速度
  geometry_msgs::msg::Twist current_;
  if (open_loop_) {
    current_ = last_cmd_;
  } else {
    current_ = odom_smoother_->getTwist();
  }

  // Apply absolute velocity restrictions to the command  // 对命令应用绝对速度限制
  command_->linear.x = std::clamp(command_->linear.x, min_velocities_[0], max_velocities_[0]);
  command_->linear.y = std::clamp(command_->linear.y, min_velocities_[1], max_velocities_[1]);
  command_->angular.z = std::clamp(command_->angular.z, min_velocities_[2], max_velocities_[2]);

  // Find if any component is not within the acceleration constraints. If so, store the most
  // significant scale factor to apply to the vector <dvx, dvy, dvw>, eta, to reduce all axes
  // proportionally to follow the same direction, within change of velocity bounds.
  // In case eta reduces another axis out of its own limit, apply accel constraint to guarantee
  // output is within limits, even if it deviates from requested command slightly.

  // 查找是否有任何分量不在加速度约束范围内。如果是，则存储最显著的缩放因子
  // 以便将矢量 <dvx, dvy, dvw> 按比例缩小，eta，以在速度变化范围内沿相同方向减小所有轴。
  // 如果 eta 使另一个轴超出其自身限制，则应用加速度约束以确保输出在限制范围内，
  // 即使它略微偏离请求的命令。
  double eta = 1.0;
  if (scale_velocities_) {
    double curr_eta = -1.0;

    curr_eta = findEtaConstraint(
      current_.linear.x, command_->linear.x, max_accels_[0], max_decels_[0]);
    if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
      eta = curr_eta;
    }

    curr_eta = findEtaConstraint(
      current_.linear.y, command_->linear.y, max_accels_[1], max_decels_[1]);
    if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
      eta = curr_eta;
    }

    curr_eta = findEtaConstraint(
      current_.angular.z, command_->angular.z, max_accels_[2], max_decels_[2]);
    if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
      eta = curr_eta;
    }
  }

  cmd_vel->linear.x = applyConstraints(
    current_.linear.x, command_->linear.x, max_accels_[0], max_decels_[0], eta);
  cmd_vel->linear.y = applyConstraints(
    current_.linear.y, command_->linear.y, max_accels_[1], max_decels_[1], eta);
  cmd_vel->angular.z = applyConstraints(
    current_.angular.z, command_->angular.z, max_accels_[2], max_decels_[2], eta);
  last_cmd_ = *cmd_vel;

  // Apply deadband restrictions & publish  // 应用死区限制并发布命令
  cmd_vel->linear.x = fabs(cmd_vel->linear.x) < deadband_velocities_[0] ? 0.0 : cmd_vel->linear.x;
  cmd_vel->linear.y = fabs(cmd_vel->linear.y) < deadband_velocities_[1] ? 0.0 : cmd_vel->linear.y;
  cmd_vel->angular.z = fabs(cmd_vel->angular.z) <
    deadband_velocities_[2] ? 0.0 : cmd_vel->angular.z;


  // RCLCPP_WARN(get_logger(), "cmd_vel->linear.x: %f, cmd_vel->linear.y: %f,  cmd_vel->angular.z: %f",  
  // cmd_vel->linear.x,  cmd_vel->linear.y, cmd_vel->angular.z);

  // //由于gazebo中的后退时前轮转角的方向有问题，故加入这段代码
  // if (cmd_vel->linear.x < 0.0)
  // {
  //   cmd_vel->angular.z = -cmd_vel->angular.z;
  // }

  
  smoothed_cmd_pub_->publish(std::move(cmd_vel));
}

rcl_interfaces::msg::SetParametersResult
VelocitySmoother::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "smoothing_frequency") {
        smoothing_frequency_ = parameter.as_double();
        if (timer_) {
          timer_->cancel();
          timer_.reset();
        }

        double timer_duration_ms = 1000.0 / smoothing_frequency_;
        timer_ = create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(timer_duration_ms)),
          std::bind(&VelocitySmoother::smootherTimer, this));
      } else if (name == "velocity_timeout") {
        velocity_timeout_ = rclcpp::Duration::from_seconds(parameter.as_double());
      } else if (name == "odom_duration") {
        odom_duration_ = parameter.as_double();
        odom_smoother_ =
          std::make_unique<nav2_util::OdomSmoother>(
          shared_from_this(), odom_duration_, odom_topic_);
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE_ARRAY) {
      if (parameter.as_double_array().size() != 3) {
        RCLCPP_WARN(get_logger(), "Invalid size of parameter %s. Must be size 3", name.c_str());
        result.successful = false;
        break;
      }

      if (name == "max_velocity") {
        max_velocities_ = parameter.as_double_array();
      } else if (name == "min_velocity") {
        min_velocities_ = parameter.as_double_array();
      } else if (name == "max_accel") {
        for (unsigned int i = 0; i != 3; i++) {
          if (parameter.as_double_array()[i] < 0.0) {
            RCLCPP_WARN(
              get_logger(),
              "Negative values set of acceleration! These should be positive to speed up!");
            result.successful = false;
          }
        }
        max_accels_ = parameter.as_double_array();
      } else if (name == "max_decel") {
        for (unsigned int i = 0; i != 3; i++) {
          if (parameter.as_double_array()[i] > 0.0) {
            RCLCPP_WARN(
              get_logger(),
              "Positive values set of deceleration! These should be negative to slow down!");
            result.successful = false;
          }
        }
        max_decels_ = parameter.as_double_array();
      } else if (name == "deadband_velocity") {
        deadband_velocities_ = parameter.as_double_array();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "feedback") {
        if (parameter.as_string() == "OPEN_LOOP") {
          open_loop_ = true;
          odom_smoother_.reset();
        } else if (parameter.as_string() == "CLOSED_LOOP") {
          open_loop_ = false;
          odom_smoother_ =
            std::make_unique<nav2_util::OdomSmoother>(
            shared_from_this(), odom_duration_, odom_topic_);
        } else {
          RCLCPP_WARN(
            get_logger(), "Invalid feedback_type, options are OPEN_LOOP and CLOSED_LOOP.");
          result.successful = false;
          break;
        }
      } else if (name == "odom_topic") {
        odom_topic_ = parameter.as_string();
        odom_smoother_ =
          std::make_unique<nav2_util::OdomSmoother>(
          shared_from_this(), odom_duration_, odom_topic_);
      }
    }
  }

  return result;
}

}  // namespace nav2_velocity_smoother

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_velocity_smoother::VelocitySmoother)

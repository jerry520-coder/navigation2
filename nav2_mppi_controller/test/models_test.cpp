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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"

// Tests model classes with methods

// 初始化ROS，这在使用ROS进行开发时需要
class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);} // 构造函数中初始化ROS
  ~RosLockGuard() {rclcpp::shutdown();} // 析构函数中关闭ROS
};
RosLockGuard g_rclcpp; // 创建一个全局对象以确保ROS在程序的生命周期内初始化和关闭

using namespace mppi::models;  // 使用mppi命名空间中的models模块，方便后续调用

// 定义第一个测试套件，测试控制序列
TEST(ModelsTest, ControlSequenceTest)
{
  // 创建一个ControlSequence对象
  ControlSequence sequence;
  sequence.vx = xt::ones<float>({10}); // 给速度向量vx赋值为10个1
  sequence.vy = xt::ones<float>({10}); // 给速度向量vy赋值为10个1
  sequence.wz = xt::ones<float>({10}); // 给角速度向量wz赋值为10个1

  // 验证序列中的元素是否正确
  EXPECT_EQ(sequence.vx(4), 1); // 验证vx的第5个元素是否为1
  EXPECT_EQ(sequence.vy(4), 1); // 验证vy的第5个元素是否为1
  EXPECT_EQ(sequence.wz(4), 1); // 验证wz的第5个元素是否为1

  sequence.reset(20); // 重置序列大小为20

  // 验证重置后的序列
  EXPECT_EQ(sequence.vx(4), 0); // 验证重置后vx的第5个元素是否为0
  EXPECT_EQ(sequence.vy(4), 0); // 验证重置后vy的第5个元素是否为0
  EXPECT_EQ(sequence.wz(4), 0); // 验证重置后wz的第5个元素是否为0
  EXPECT_EQ(sequence.vx.shape(0), 20u); // 验证重置后vx的大小
  EXPECT_EQ(sequence.vy.shape(0), 20u); // 验证重置后vy的大小
  EXPECT_EQ(sequence.wz.shape(0), 20u); // 验证重置后wz的大小
}

// 定义第二个测试套件，测试路径
TEST(ModelsTest, PathTest)
{
  Path path;
  path.x = xt::ones<float>({10}); // 给路径的x坐标赋值
  path.y = xt::ones<float>({10}); // 给路径的y坐标赋值
  path.yaws = xt::ones<float>({10}); // 给路径的偏航角赋值

  // 同样的验证方法
  EXPECT_EQ(path.x(4), 1);
  EXPECT_EQ(path.y(4), 1);
  EXPECT_EQ(path.yaws(4), 1);

  path.reset(20); // 重置路径大小为20

  // 验证重置后的路径
  EXPECT_EQ(path.x(4), 0);
  EXPECT_EQ(path.y(4), 0);
  EXPECT_EQ(path.yaws(4), 0);
  EXPECT_EQ(path.x.shape(0), 20u);
  EXPECT_EQ(path.y.shape(0), 20u);
  EXPECT_EQ(path.yaws.shape(0), 20u);
}

// 定义第三个测试套件，测试状态
TEST(ModelsTest, StateTest)
{
  State state;
  state.vx = xt::ones<float>({10, 10}); // 为状态的各个向量赋值，这里包括速度和控制速度
  state.vy = xt::ones<float>({10, 10});
  state.wz = xt::ones<float>({10, 10});
  state.cvx = xt::ones<float>({10, 10});
  state.cvy = xt::ones<float>({10, 10});
  state.cwz = xt::ones<float>({10, 10});

  // 验证状态
  EXPECT_EQ(state.cvx(4), 1);
  EXPECT_EQ(state.cvy(4), 1);
  EXPECT_EQ(state.cwz(4), 1);
  EXPECT_EQ(state.vx(4), 1);
  EXPECT_EQ(state.vy(4), 1);
  EXPECT_EQ(state.wz(4), 1);

  state.reset(20, 40); // 重置状态的大小

  // 验证重置后的状态
  EXPECT_EQ(state.cvx(4), 0);
  EXPECT_EQ(state.cvy(4), 0);
  EXPECT_EQ(state.cwz(4), 0);
  EXPECT_EQ(state.vx(4), 0);
  EXPECT_EQ(state.vy(4), 0);
  EXPECT_EQ(state.wz(4), 0);
  EXPECT_EQ(state.cvx.shape(0), 20u); // 验证大小
  EXPECT_EQ(state.cvy.shape(0), 20u);
  EXPECT_EQ(state.cwz.shape(0), 20u);
  EXPECT_EQ(state.cvx.shape(1), 40u);
  EXPECT_EQ(state.cvy.shape(1), 40u);
  EXPECT_EQ(state.cwz.shape(1), 40u);
  EXPECT_EQ(state.vx.shape(0), 20u);
  EXPECT_EQ(state.vy.shape(0), 20u);
  EXPECT_EQ(state.wz.shape(0), 20u);
  EXPECT_EQ(state.vx.shape(1), 40u);
  EXPECT_EQ(state.vy.shape(1), 40u);
  EXPECT_EQ(state.wz.shape(1), 40u);
}

// 定义第四个测试套件，测试轨迹
TEST(ModelsTest, TrajectoriesTest)
{
  Trajectories trajectories;
  trajectories.x = xt::ones<float>({10, 10}); // 为轨迹的各个向量赋值
  trajectories.y = xt::ones<float>({10, 10});
  trajectories.yaws = xt::ones<float>({10, 10});

  // 验证轨迹
  EXPECT_EQ(trajectories.x(4), 1);
  EXPECT_EQ(trajectories.y(4), 1);
  EXPECT_EQ(trajectories.yaws(4), 1);

  trajectories.reset(20, 40); // 重置轨迹的大小

  // 验证重置后的轨迹
  EXPECT_EQ(trajectories.x(4), 0);
  EXPECT_EQ(trajectories.y(4), 0);
  EXPECT_EQ(trajectories.yaws(4), 0);
  EXPECT_EQ(trajectories.x.shape(0), 20u); // 验证大小
  EXPECT_EQ(trajectories.y.shape(0), 20u);
  EXPECT_EQ(trajectories.yaws.shape(0), 20u);
  EXPECT_EQ(trajectories.x.shape(1), 40u);
  EXPECT_EQ(trajectories.y.shape(1), 40u);
  EXPECT_EQ(trajectories.yaws.shape(1), 40u);
}
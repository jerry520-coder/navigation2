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

#include "nav2_mppi_controller/tools/noise_generator.hpp"

#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{

void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic,
  const std::string & name, ParametersHandler * param_handler)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;

  auto getParam = param_handler->getParamGetter(name);
  getParam(regenerate_noises_, "regenerate_noises", false);

  if (regenerate_noises_) {
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::shutdown()
{
  active_ = false;
  ready_ = true;
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises()
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  {
  // 使用互斥锁保护共享资源 ready_
  // 表示下一次迭代的噪声数据已经准备好
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }
  noise_cond_.notify_all(); // 通知所有等待的线程，生成下一次迭代的噪声数据已经准备好
}


void NoiseGenerator::setNoisedControls(
  models::State & state,                             // 状态对象，用于存储带有噪声的控制参数
  const models::ControlSequence & control_sequence)  // 控制序列，包含原始的控制参数
{
  // 使用互斥锁保护共享资源
  std::unique_lock<std::mutex> guard(noise_lock_);

  // 将噪声添加到 x 方向上的速度控制参数，并存储到 State 对象的 cvx 成员变量中
  xt::noalias(state.cvx) = control_sequence.vx + noises_vx_;  //xt::noalias 可以告诉编译器避免创建这个临时对象，直接在 A 上进行修改。

  // 将噪声添加到 y 方向上的速度控制参数，并存储到 State 对象的 cvy 成员变量中
  xt::noalias(state.cvy) = control_sequence.vy + noises_vy_;

  // 将噪声添加到 z 轴上的角速度控制参数，并存储到 State 对象的 cwz 成员变量中
  xt::noalias(state.cwz) = control_sequence.wz + noises_wz_;
}


void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Recompute the noises on reset, initialization, and fallback
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    xt::noalias(noises_vx_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_vy_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_wz_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    ready_ = true;
  }

// 决定是重新生成噪声还是继续使用现有的噪声来生成带噪声的控制信号
  if (regenerate_noises_) {
    noise_cond_.notify_all(); //唤醒所有正在等待 noise_cond_ 的线程。这意味着，如果有其他线程正在等待噪声的生成，它们现在可以继续执行，因为噪声将被重新生成。
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::noiseThread()
{
  // 这个 do-while 循环会在 active_ 为 true 的情况下一直执行
  do {
    // 创建一个独占的互斥锁，用于在生成噪声控制信号时保护共享资源
    std::unique_lock<std::mutex> guard(noise_lock_);
    // 等待，直到 ready_ 变为 true 才继续执行
    noise_cond_.wait(guard, [this]() {return ready_;});  //这个函数调用会在 ready_ 变为 true 时继续执行，否则会一直等待下去。
    // 将 ready_ 置为 false，因为已经开始生成噪声控制信号
    ready_ = false;
    // 调用生成噪声控制信号的函数
    generateNoisedControls();
  } while (active_);  // 如果 active_ 为 true，则继续循环
}

void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;  // 获取设置的引用，方便后面使用

  // 生成随机噪声 noises_vx_，大小为 [batch_size, time_steps]，均值为 0，标准差为 s.sampling_std.vx
  xt::noalias(noises_vx_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.vx); //从正态分布中生成随机数，并将结果存储在类的成员变量中

  // 生成随机噪声 noises_wz_，大小为 [batch_size, time_steps]，均值为 0，标准差为 s.sampling_std.wz
  xt::noalias(noises_wz_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.wz);

  if (is_holonomic_) {  // 如果是 holonomic 的话
    // 生成随机噪声 noises_vy_，大小为 [batch_size, time_steps]，均值为 0，标准差为 s.sampling_std.vy
    xt::noalias(noises_vy_) = xt::random::randn<float>(
      {s.batch_size, s.time_steps}, 0.0f,
      s.sampling_std.vy);
  }

// xt::random::randn<float>() 是一个函数，用于从正态分布中生成随机数。
// {s.batch_size, s.time_steps} 是生成的矩阵的大小，其中 s.batch_size 是批量大小（batch size），s.time_steps 是时间步数。
// 0.0f 是这些随机数的均值，即生成的随机数围绕着 0.0f 进行分布。
// s.sampling_std.vx 是这些随机数的标准差，决定了随机数分布的“宽度”或“幅度”。
}

}  // namespace mppi

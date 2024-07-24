// Copyright (c) 2019 Intel Corporation
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
#include <iostream>
#include <cstdlib>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_utils.hpp"

using std::cerr;
using namespace std::chrono_literals;

void usage()
{
  cerr << "Invalid command line.\n\n";// 输出“命令行无效”的错误提示
  // 提供关于程序的操作指南
  cerr << "This command will take a set of unconfigured lifecycle nodes through the\n";
  cerr << "CONFIGURED to the ACTIVATED state\n";
  cerr << "The nodes are brought up in the order listed on the command line\n\n";
  cerr << "Usage:\n";
  cerr << " > lifecycle_startup <node name> ...\n";  // 显示如何正确使用命令行参数启动生命周期节点

  // 结束程序执行，并返回错误码1
  std::exit(1);
}

int main(int argc, char * argv[]) //argv[0] 通常是程序的名称，而从 argv[1] 开始则是用户输入的参数。
{
  if (argc == 1) {
    usage();
  }
  rclcpp::init(0, nullptr); //表示对命令行参数的解析由程序自行管理，而非ROS 2库。

// 用于启动指定的生命周期节点。它接收一个字符串向量，这个向量是由 argv 指针加1到 argc 生成的，即传递给程序的所有用户定义参数。
// 10s 表示等待或超时时间参数，使用了C++中的字面量持续时间表示法，这里指定的是10秒。
  nav2_util::startup_lifecycle_nodes(
    std::vector<std::string>(argv + 1, argv + argc),
    10s);
  rclcpp::shutdown();
}

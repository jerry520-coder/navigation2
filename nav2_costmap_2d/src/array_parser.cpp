/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 *
 * author: Dave Hershberger
 */

#include <cstdio>  // for EOF
#include <string>
#include <sstream>
#include <vector>

namespace nav2_costmap_2d
{

/** @brief Parse a vector of vectors of floats from a string.
 * @param input
 * @param error_return
 * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...] 
 * 
 * @note vvf 是一个变量名，通常这个名字是 std::vector<std::vector<float>> 类型的缩写
 * */
std::vector<std::vector<float>> parseVVF(const std::string & input, std::string & error_return)
{
  std::vector<std::vector<float>> result; // 存储最终结果的二维向量

  std::stringstream input_ss(input); // 使用字符串流进行字符串处理
  int depth = 0; // 用于记录当前解析深度
  std::vector<float> current_vector; // 当前解析的一维向量

  // 循环读取每个字符，直到文件结束
  while (!!input_ss && !input_ss.eof()) {
    switch (input_ss.peek()) { // 检查下一个字符，但不从流中移除
      case EOF:
        break;
      case '[':
        depth++; // 增加嵌套深度
        if (depth > 2) {
          error_return = "Array depth greater than 2"; // 深度超过2则返回错误
          return result;
        }
        input_ss.get(); // 读取'['字符
        current_vector.clear(); // 清空当前向量，准备读取新的一组数据
        break;
      case ']':
        depth--; // 减少嵌套深度
        if (depth < 0) {
          error_return = "More close ] than open ["; // 如果关闭的括号多于开启的括号，返回错误
          return result;
        }
        input_ss.get(); // 读取']'字符
        if (depth == 1) {
          result.push_back(current_vector); // 将当前向量添加到结果中
        }
        break;
        // 忽略逗号、空格和制表符
      case ',':
      case ' ':
      case '\t':
        input_ss.get(); 
        break;
      default:  // All other characters should be part of the numbers.。其他字符应为数字的一部分
        if (depth != 2) {
          std::stringstream err_ss;
          err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
          error_return = err_ss.str(); // 如果数字不在深度2，返回错误
          return result;
        }
        float value;
        input_ss >> value; // 从流中读取一个浮点数
        if (!!input_ss) {
          current_vector.push_back(value); // 如果读取成功，将数值添加到当前向量
        }
        break;
    }
  }

  // 检查是否所有的括号都正确关闭
  if (depth != 0) {
    error_return = "Unterminated vector string."; // 如果有未关闭的括号，返回错误
  } else {
    error_return = ""; // 没有错误，清空错误信息
  }

  return result; // 返回解析的结果
}


}  // end namespace nav2_costmap_2d

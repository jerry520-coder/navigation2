// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef NAV2_SMAC_PLANNER__NODE_BASIC_HPP_
#define NAV2_SMAC_PLANNER__NODE_BASIC_HPP_

#include <math.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <functional>
#include <queue>
#include <memory>
#include <utility>
#include <limits>

#include "ompl/base/StateSpace.h"

#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/node_2d.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/collision_checker.hpp"

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::NodeBasic
 * @brief NodeBasic implementation for priority queue insertion。优先队列插入的 NodeBasic 实现
 */
template<typename NodeT>
class NodeBasic
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::NodeBasic
   * @param index The index of this node for self-reference。该节点的索引，用于自引用
   */
  explicit NodeBasic(const unsigned int index)
  : index(index),
    graph_node_ptr(nullptr)
  {
  }

  /**
   * @brief Take a NodeBasic and populate it with any necessary state
   * cached in the queue for NodeT.
   * @param node NodeT ptr to populate metadata into NodeBasic
   */

  /**
 * @brief 采用一个 NodeBasic， 并用 NodeT 的queue中缓存的任何必要状态填充它。
 * @param node NodeT ptr to populate metadata into NodeBasic。用于向 NodeBasic 填充元数据的 NodeT 指针。
 */
  void populateSearchNode(NodeT * & node);

  /**
   * @brief Take a NodeBasic and populate it with any necessary state
   * cached in the queue for NodeTs.
   * @param node Search node (basic) object to initialize internal node
   * with state
   */

  /**
 * @brief 采用一个 NodeBasic，并用 NodeTs 的queue中缓存的任何必要状态填充它。
 * @param node Search node (basic) object to initialize internal node with state。用于用状态初始化内部节点的搜索节点（基本）对象。
 */
  void processSearchNode();

  typename NodeT::Coordinates pose;  // Used by NodeHybrid and NodeLattice
  NodeT * graph_node_ptr;
  MotionPrimitive * prim_ptr;  // Used by NodeLattice
  unsigned int index, motion_index;
  bool backward;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NODE_BASIC_HPP_

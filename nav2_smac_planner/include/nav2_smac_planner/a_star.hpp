// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
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

#ifndef NAV2_SMAC_PLANNER__A_STAR_HPP_
#define NAV2_SMAC_PLANNER__A_STAR_HPP_

#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>
#include "Eigen/Core"

#include "nav2_costmap_2d/costmap_2d.hpp"

#include "nav2_smac_planner/thirdparty/robin_hood.h"
#include "nav2_smac_planner/analytic_expansion.hpp"
#include "nav2_smac_planner/node_2d.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/node_basic.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/constants.hpp"

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::AStarAlgorithm
 * @brief An A* implementation for planning in a costmap. Templated based on the Node type.
 */
template<typename NodeT>
class AStarAlgorithm
{
public:
  typedef NodeT * NodePtr;
  typedef robin_hood::unordered_node_map<unsigned int, NodeT> Graph;
  typedef std::vector<NodePtr> NodeVector;
  typedef std::pair<float, NodeBasic<NodeT>> NodeElement;
  typedef typename NodeT::Coordinates Coordinates;
  typedef typename NodeT::CoordinateVector CoordinateVector;
  typedef typename NodeVector::iterator NeighborIterator;
  typedef std::function<bool (const unsigned int &, NodeT * &)> NodeGetter;

  /**
   * @struct nav2_smac_planner::NodeComparator
   * @brief Node comparison for priority queue sorting
   */
  struct NodeComparator
  {
    bool operator()(const NodeElement & a, const NodeElement & b) const
    {
      return a.first > b.first;
    }
  };

  typedef std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator> NodeQueue;

  /**
   * @brief A constructor for nav2_smac_planner::AStarAlgorithm
   */
  explicit AStarAlgorithm(const MotionModel & motion_model, const SearchInfo & search_info);

  /**
   * @brief A destructor for nav2_smac_planner::AStarAlgorithm
   */
  ~AStarAlgorithm();

  /**
   * @brief Initialization of the planner with defaults
   * @param allow_unknown Allow search in unknown space, good for navigation while mapping
   * @param max_iterations Maximum number of iterations to use while expanding search
   * @param max_on_approach_iterations Maximum number of iterations before returning a valid
   * path once within thresholds to refine path
   * comes at more compute time but smoother paths.
   * @param max_planning_time Maximum time (in seconds) to wait for a plan, createPath returns
   * false after this timeout
   */

  /**
 * @brief 使用默认设置初始化规划器
 * @param allow_unknown 允许在未知空间中搜索，适用于边建图边导航的情况
 * @param max_iterations 扩展搜索时使用的最大迭代次数
 * @param max_on_approach_iterations 在接近目标阈值范围内，返回有效路径前的最大迭代次数，
 * 使用更多的计算时间但能得到更平滑的路径。
 * @param max_planning_time 等待规划结果的最大时间（以秒为单位），超时后createPath返回false
 */
  void initialize(
    const bool & allow_unknown,
    int & max_iterations,
    const int & max_on_approach_iterations,
    const double & max_planning_time,
    const float & lookup_table_size,
    const unsigned int & dim_3_size);

  /**
   * @brief Creating path from given costmap, start, and goal
   * @param path Reference to a vector of indicies of generated path
   * @param num_iterations Reference to number of iterations to create plan
   * @param tolerance Reference to tolerance in costmap nodes
   * @return if plan was successful
   */

  /**
 * @brief 从给定的成本地图、起点和终点创建路径
 * @param path 对生成路径的索引向量的引用
 * @param num_iterations 对创建计划所需迭代次数的引用
 * @param tolerance 对成本地图节点的容差的引用
 * @return 如果规划成功创建，则返回true
 */
  bool createPath(CoordinateVector & path, int & num_iterations, const float & tolerance);

  /**
   * @brief Sets the collision checker to use
   * @param collision_checker Collision checker to use for checking state validity
   * @note 设置碰撞检查器，用于在 A* 算法中判断节点是否可通过
   */
  void setCollisionChecker(GridCollisionChecker * collision_checker);

  /**
   * @brief Set the goal for planning, as a node index
   * @param mx The node X index of the goal
   * @param my The node Y index of the goal
   * @param dim_3 The node dim_3 index of the goal
   */

  /**
 * @brief 设置规划目标，作为节点索引
 * @param mx 目标的节点X索引
 * @param my 目标的节点Y索引
 * @param dim_3 目标的节点dim_3索引
 */
  void setGoal(
    const unsigned int & mx,
    const unsigned int & my,
    const unsigned int & dim_3);

  /**
   * @brief Set the starting pose for planning, as a node index
   * @param mx The node X index of the goal
   * @param my The node Y index of the goal
   * @param dim_3 The node dim_3 index of the goal.表示目标节点的第三个维度的索引
   */
  void setStart(
    const unsigned int & mx,
    const unsigned int & my,
    const unsigned int & dim_3);

  /**
   * @brief Get maximum number of iterations to plan
   * @return Reference to Maximum iterations parameter
   */
  int & getMaxIterations();

  /**
   * @brief Get pointer reference to starting node
   * @return Node pointer reference to starting node
   */
  NodePtr & getStart();

  /**
   * @brief Get pointer reference to goal node
   * @return Node pointer reference to goal node
   */
  NodePtr & getGoal();

  /**
   * @brief Get maximum number of on-approach iterations after within threshold
   * @return Reference to Maximum on-appraoch iterations parameter
   */
  int & getOnApproachMaxIterations();

  /**
   * @brief Get tolerance, in node nodes
   * @return Reference to tolerance parameter
   */
  float & getToleranceHeuristic();

  /**
   * @brief Get size of graph in X
   * @return Size in X
   */
  unsigned int & getSizeX();

  /**
   * @brief Get size of graph in Y
   * @return Size in Y
   */
  unsigned int & getSizeY();

  /**
   * @brief Get number of angle quantization bins (SE2) or Z coordinate  (XYZ)
   * @return Number of angle bins / Z dimension
   */

  /**
 * @brief 获取角度量化的bins数（SE2情况下）或Z坐标（XYZ情况下）
 * @return 角度箱数或Z维度的数量
 */
  unsigned int & getSizeDim3();

protected:
  /**
   * @brief Get pointer to next goal in open set
   * @return Node pointer reference to next heuristically scored node
   */
  inline NodePtr getNextNode();

  /**
   * @brief Add a node to the open set
   * @param cost The cost to sort into the open set of the node
   * @param node Node pointer reference to add to open set
   */
  inline void addNode(const float & cost, NodePtr & node);

  /**
   * @brief Adds node to graph
   * @param index Node index to add
   */
  inline NodePtr addToGraph(const unsigned int & index);

  /**
   * @brief Check if this node is the goal node
   * @param node Node pointer to check if its the goal node
   * @return if node is goal
   */
  inline bool isGoal(NodePtr & node);

  /**
   * @brief Get cost of heuristic of node
   * @param node Node pointer to get heuristic for
   * @return Heuristic cost for node
   */
  inline float getHeuristicCost(const NodePtr & node);

  /**
   * @brief Check if inputs to planner are valid
   * @return Are valid
   */
  inline bool areInputsValid();

  /**
   * @brief Clear hueristic queue of nodes to search
   */
  inline void clearQueue();

  /**
   * @brief Clear graph of nodes searched
   */
  inline void clearGraph();

  int _timing_interval = 5000;

  bool _traverse_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  double _max_planning_time;
  float _tolerance;
  unsigned int _x_size;
  unsigned int _y_size;
  unsigned int _dim3_size;
  SearchInfo _search_info;

  Coordinates _goal_coordinates;
  NodePtr _start;
  NodePtr _goal;

  Graph _graph;
  NodeQueue _queue;

  MotionModel _motion_model;
  NodeHeuristicPair _best_heuristic_node;

  GridCollisionChecker * _collision_checker;
  nav2_costmap_2d::Costmap2D * _costmap;
  std::unique_ptr<AnalyticExpansion<NodeT>> _expander;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__A_STAR_HPP_

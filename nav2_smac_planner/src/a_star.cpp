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

#include <omp.h>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <algorithm>
#include <limits>
#include <type_traits>
#include <chrono>
#include <thread>
#include <utility>
#include <vector>

#include "nav2_smac_planner/a_star.hpp"
using namespace std::chrono;  // NOLINT

namespace nav2_smac_planner
{

template<typename NodeT>
AStarAlgorithm<NodeT>::AStarAlgorithm(
  const MotionModel & motion_model,
  const SearchInfo & search_info)
: _traverse_unknown(true),
  _max_iterations(0),
  _max_planning_time(0),
  _x_size(0),
  _y_size(0),
  _search_info(search_info),
  _goal_coordinates(Coordinates()),
  _start(nullptr),
  _goal(nullptr),
  _motion_model(motion_model)
{
  _graph.reserve(100000);
}

template<typename NodeT>
AStarAlgorithm<NodeT>::~AStarAlgorithm()
{
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::initialize(
  const bool & allow_unknown,
  int & max_iterations,
  const int & max_on_approach_iterations,
  const double & max_planning_time,
  const float & lookup_table_size,
  const unsigned int & dim_3_size)
{
  _traverse_unknown = allow_unknown;  // 设置是否允许穿越未知区域
  _max_iterations = max_iterations;  // 设置最大迭代次数
  _max_on_approach_iterations = max_on_approach_iterations;  // 设置接近目标时的最大迭代次数
  _max_planning_time = max_planning_time;  // 设置最大规划时间
  // 调用 NodeT 类型的静态方法 precomputeDistanceHeuristic，预计算距离启发式
  NodeT::precomputeDistanceHeuristic(lookup_table_size, _motion_model, dim_3_size, _search_info); //static 函数
  _dim3_size = dim_3_size;  // 设置第三维度的大小

  // 创建一个 AnalyticExpansion 对象，用于处理 A* 算法的分析扩展部分
  _expander = std::make_unique<AnalyticExpansion<NodeT>>(
    _motion_model, _search_info, _traverse_unknown, _dim3_size);
}

//模版特化
template<>
void AStarAlgorithm<Node2D>::initialize( 
  const bool & allow_unknown,
  int & max_iterations,
  const int & max_on_approach_iterations,
  const double & max_planning_time,
  const float & /*lookup_table_size*/,
  const unsigned int & dim_3_size)
{
  _traverse_unknown = allow_unknown;
  _max_iterations = max_iterations;
  _max_on_approach_iterations = max_on_approach_iterations;
  _max_planning_time = max_planning_time;

  if (dim_3_size != 1) {
    throw std::runtime_error("Node type Node2D cannot be given non-1 dim 3 quantization."); //节点类型 Node2D 不能进行非 1 dim 3 量化。代表z轴的意思
  }
  _dim3_size = dim_3_size;
  _expander = std::make_unique<AnalyticExpansion<Node2D>>(
    _motion_model, _search_info, _traverse_unknown, _dim3_size);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setCollisionChecker(GridCollisionChecker * collision_checker)
{
  _collision_checker = collision_checker;  // 存储提供的碰撞检查器对象的引用
  _costmap = collision_checker->getCostmap();  // 从碰撞检查器获取成本地图的引用

  // 获取成本地图的尺寸（X和Y方向的单元格数量）
  unsigned int x_size = _costmap->getSizeInCellsX();
  unsigned int y_size = _costmap->getSizeInCellsY();

  // 清除当前图的数据，为新数据做准备
  clearGraph();

  // 如果当前图的尺寸与成本地图的尺寸不匹配，则更新尺寸并重新初始化运动模型
  if (getSizeX() != x_size || getSizeY() != y_size) {
    _x_size = x_size;  // 更新X方向的尺寸
    _y_size = y_size;  // 更新Y方向的尺寸

    // 调用 NodeT 类的静态方法来初始化运动模型，这通常涉及到根据新尺寸计算节点间的可能连接或成本
    NodeT::initMotionModel(_motion_model, _x_size, _y_size, _dim3_size, _search_info);
  }

  // 将新的碰撞检查器设置给扩展器对象
  _expander->setCollisionChecker(collision_checker);
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr AStarAlgorithm<NodeT>::addToGraph(
  const unsigned int & index)
{
   // 使用_graph查找index对应的节点
  auto iter = _graph.find(index);
   // 如果找到该节点，则返回该节点的引用
  if (iter != _graph.end()) {
    return &(iter->second);
  }

// 如果没有找到该节点，则使用emplace方法向图中添加一个新的节点
  // emplace用于在_map_中插入一个新的键值对，避免不必要的复制
  // 第一个参数是键（index），第二个参数是新建的NodeT对象，使用index作为构造函数参数
  // 返回值是指向新插入元素的引用
  // 由于返回的是pair类型，我们只关心.second，即NodeT对象
  return &(_graph.emplace(index, NodeT(index)).first->second);
}

template<>
void AStarAlgorithm<Node2D>::setStart(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  if (dim_3 != 0) {
    throw std::runtime_error("Node type Node2D cannot be given non-zero starting dim 3.");
  }
  _start = addToGraph(Node2D::getIndex(mx, my, getSizeX()));
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setStart(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  _start = addToGraph(NodeT::getIndex(mx, my, dim_3));
  _start->setPose(
    Coordinates(
      static_cast<float>(mx),
      static_cast<float>(my),
      static_cast<float>(dim_3)));
}

template<>
void AStarAlgorithm<Node2D>::setGoal(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  if (dim_3 != 0) {
    throw std::runtime_error("Node type Node2D cannot be given non-zero goal dim 3.");
  }

  _goal = addToGraph(Node2D::getIndex(mx, my, getSizeX()));
  _goal_coordinates = Node2D::Coordinates(mx, my);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setGoal(
  const unsigned int & mx,  // x坐标索引
  const unsigned int & my,  // y坐标索引
  const unsigned int & dim_3)  // 第三维的索引（例如高度或时间戳）
{
  // 根据提供的坐标索引计算目标点在图中的索引，并将其添加到图中
  _goal = addToGraph(NodeT::getIndex(mx, my, dim_3));

  // 创建一个包含目标点坐标的NodeT类型的坐标结构
  typename NodeT::Coordinates goal_coords(
    static_cast<float>(mx),  // 将x坐标转换为浮点数
    static_cast<float>(my),  // 将y坐标转换为浮点数
    static_cast<float>(dim_3));  // 将第三维的索引转换为浮点数

  // 如果没有缓存障碍物启发式值，或者当前目标坐标与之前的目标坐标不同，则重置障碍物启发式值
  if (!_search_info.cache_obstacle_heuristic || goal_coords != _goal_coordinates) {
    if (!_start) {
      // 如果起点未设置，则在设置目标之前抛出运行时错误
      throw std::runtime_error("Start must be set before goal.");
    }

    // 重置障碍物启发式值，基于起点和目标点的坐标
    NodeT::resetObstacleHeuristic(_costmap, _start->pose.x, _start->pose.y, mx, my);
  }

  // 更新目标点坐标的缓存
  _goal_coordinates = goal_coords;

  // 设置目标点的位置
  _goal->setPose(_goal_coordinates);
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::areInputsValid()
{
  // 检查图是否已填充
  if (_graph.empty()) {
    // 如果图为空，则抛出运行时错误
    throw std::runtime_error("Failed to compute path, no costmap given.");
  }

  // 检查起点和终点是否已填充
  if (!_start || !_goal) {
    // 如果起点或终点未定义，则抛出运行时错误
    throw std::runtime_error("Failed to compute path, no valid start or goal given.");
  }

  // 检查终点是否有效
  if (getToleranceHeuristic() < 0.001 &&
      !_goal->isNodeValid(_traverse_unknown, _collision_checker))
  {
    // 如果终点在没有容忍度的情况下被占据，则抛出运行时错误
    throw std::runtime_error("Failed to compute path, goal is occupied with no tolerance.");
  }

  // 检查起点是否有效
  if (!_start->isNodeValid(_traverse_unknown, _collision_checker)) {
    // 如果起点位于致命空间，则抛出运行时错误，因为无法创建可行的规划
    throw std::runtime_error("Starting point in lethal space! Cannot create feasible plan.");
  }

  // 如果所有输入都有效，则返回true
  return true;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::createPath(
  CoordinateVector & path, int & iterations,
  const float & tolerance)
{
  // 获取算法开始时间
  steady_clock::time_point start_time = steady_clock::now();
  _tolerance = tolerance; // 设置容忍度，用于判断何时停止算法
  _best_heuristic_node = {std::numeric_limits<float>::max(), 0}; // 初始化最优启发式节点
  clearQueue(); // 清空优先队列

  // 验证输入参数是否有效
  if (!areInputsValid()) {
    return false;
  }

  // 0) 将起点加入开放集 the open set
  addNode(0.0, getStart());
  getStart()->setAccumulatedCost(0.0); // 设置起点的累积成本为 0

  // Optimization：预分配所有变量
  NodePtr current_node = nullptr; // 当前节点
  NodePtr neighbor = nullptr; // 邻居节点
  NodePtr expansion_result = nullptr; // 扩展结果
  float g_cost = 0.0; // G成本（从起点到当前节点的成本）
  NodeVector neighbors; // 邻居节点集合
  int approach_iterations = 0; // 接近迭代次数
  NeighborIterator neighbor_iterator; // 邻居迭代器
  int analytic_iterations = 0; // 分析迭代次数
  int closest_distance = std::numeric_limits<int>::max(); // 最近距离

  // 用于根据索引，返回有效且无碰撞的节点指针
  const unsigned int max_index = getSizeX() * getSizeY() * getSizeDim3(); // 计算最大索引
  NodeGetter neighborGetter =
    [&, this](const unsigned int & index, NodePtr & neighbor_rtn) -> bool
    {
      if (index >= max_index) {
        return false;
      }

      neighbor_rtn = addToGraph(index);
      return true;
    };

  // 主循环：直到达到最大迭代次数或队列为空
  while (iterations < getMaxIterations() && !_queue.empty()) {
    // 每 N 次迭代检查一次规划超时
    if (iterations % _timing_interval == 0) {
      std::chrono::duration<double> planning_duration =
        std::chrono::duration_cast<std::chrono::duration<double>>(steady_clock::now() - start_time);
      if (static_cast<double>(planning_duration.count()) >= _max_planning_time) {
        return false;
      }
    }

    // 1) 从开放集中取出成本最低的节点，移除队列。Pick Nbest from O s.t. min(f(Nbest)), remove from queue
    current_node = getNextNode();

    // 如果节点已访问，则跳过
    // 我们允许节点被多次加入队列以应对路径长度减少的情况，但每个节点只能被访问一次
    if (current_node->wasVisited()) {
      continue;
    }

    iterations++; // 迭代计数

    // 2) 标记为已访问。Mark Nbest as visited
    current_node->visited();

    // 2.1) 如果可用，使用分析扩展生成路径。Use an analytic expansion (if available) to generate a path
    expansion_result = nullptr;
    expansion_result = _expander->tryAnalyticExpansion(
      current_node, getGoal(), neighborGetter, analytic_iterations, closest_distance);
    if (expansion_result != nullptr) {
      current_node = expansion_result;
    }

    // 3) 检查是否达到目标节点，如果需要则回溯路径。Check if we're at the goal, backtrace if required
    if (isGoal(current_node)) {
      return current_node->backtracePath(path);
    } else if (_best_heuristic_node.first < getToleranceHeuristic()) {
      // 如果在容忍度范围内找到更好的节点，则继续查找直到达到接近迭代的最大次数
      // Optimization：当节点在容忍度范围内时进行查找，并在合理的范围内进行精细化处理
      approach_iterations++;
      if (approach_iterations >= getOnApproachMaxIterations()) {
        return _graph.at(_best_heuristic_node.second).backtracePath(path);
      }
    }

    // 4) 扩展当前节点的未访问邻居节点。Expand neighbors of Nbest not visited
    neighbors.clear();
    current_node->getNeighbors(neighborGetter, _collision_checker, _traverse_unknown, neighbors);

    for (neighbor_iterator = neighbors.begin();
      neighbor_iterator != neighbors.end(); ++neighbor_iterator)
    {
      neighbor = *neighbor_iterator;

      // 4.1) 计算到该邻居节点的成本。Compute the cost to go to this node
      g_cost = current_node->getAccumulatedCost() + current_node->getTraversalCost(neighbor);

      // 4.2) 如果成本低于之前的成本，我们将其定为new cost and new approach。If this is a lower cost than prior, we set this as the new cost and new approach
      if (g_cost < neighbor->getAccumulatedCost()) {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current_node;

        // 4.3) 根据启发式成本将节点加入队列。Add to queue with heuristic cost
        addNode(g_cost + getHeuristicCost(neighbor), neighbor);
      }
    }
  }

  // 如果没有搜索选项，返回最接近目标的路径（如果在容忍度内）
  if (_best_heuristic_node.first < getToleranceHeuristic()) {
    // If we run out of serach options, return the path that is closest, if within tolerance.
    // 如果我们耗尽了所有搜索选项，返回最接近的路径，前提是该路径在容忍度范围内。
    return _graph.at(_best_heuristic_node.second).backtracePath(path);
  }

  return false;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::isGoal(NodePtr & node)
{
  return node == getGoal();
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getStart()
{
  return _start;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getGoal()
{
  return _goal;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr AStarAlgorithm<NodeT>::getNextNode()
{
  NodeBasic<NodeT> node = _queue.top().second;
  _queue.pop();
  node.processSearchNode();
  return node.graph_node_ptr;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::addNode(const float & cost, NodePtr & node)
{
  NodeBasic<NodeT> queued_node(node->getIndex());
  queued_node.populateSearchNode(node);
  _queue.emplace(cost, queued_node);
}

template<typename NodeT>
float AStarAlgorithm<NodeT>::getHeuristicCost(const NodePtr & node)
{
  const Coordinates node_coords =
    NodeT::getCoords(node->getIndex(), getSizeX(), getSizeDim3());
  float heuristic = NodeT::getHeuristicCost(
    node_coords, _goal_coordinates, _costmap);

  if (heuristic < _best_heuristic_node.first) {
    _best_heuristic_node = {heuristic, node->getIndex()};
  }

  return heuristic;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::clearQueue()
{
  NodeQueue q;
  std::swap(_queue, q);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::clearGraph()
{
  Graph g;
  std::swap(_graph, g);
  _graph.reserve(100000);
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getMaxIterations()
{
  return _max_iterations;
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getOnApproachMaxIterations()
{
  return _max_on_approach_iterations;
}

template<typename NodeT>
float & AStarAlgorithm<NodeT>::getToleranceHeuristic()
{
  return _tolerance;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeX()
{
  return _x_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeY()
{
  return _y_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeDim3()
{
  return _dim3_size;
}

// Instantiate algorithm for the supported template types
template class AStarAlgorithm<Node2D>;
template class AStarAlgorithm<NodeHybrid>;
template class AStarAlgorithm<NodeLattice>;

}  // namespace nav2_smac_planner

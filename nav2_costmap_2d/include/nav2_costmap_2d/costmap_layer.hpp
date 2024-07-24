/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__COSTMAP_LAYER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_LAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

namespace nav2_costmap_2d
{

/**
 * @class CostmapLayer
 * @brief A costmap layer base class for costmap plugin layers.
 * Rather than just a layer, this object also contains an internal
 * costmap object to populate and maintain state.
 */


/**
 * @class CostmapLayer
 * @brief 一个用于代价图插件层的代价图层基类。
 * 这个对象不仅仅是一个层，它还包含一个内部的代价图对象用于填充和维护状态。
 */
class CostmapLayer : public Layer, public Costmap2D
{
public:
  /**
   * @brief CostmapLayer constructor
   */
  CostmapLayer()
  : has_extra_bounds_(false),
    extra_min_x_(1e6), extra_max_x_(-1e6),
    extra_min_y_(1e6), extra_max_y_(-1e6) {}

  /**
   * @brief If layer is discrete
   */
  bool isDiscretized()
  {
    return true;
  }

  /**
   * @brief Match the size of the master costmap
   */
  virtual void matchSize();

  /**
   * @brief Clear an are in the costmap with the given dimension
   * if invert, then clear everything except these dimensions
   */

    /**
    @brief 在代价图中清除给定尺寸的区域。
    如果设置了invert参数，那么清除除这些尺寸之外的所有区域。
  */
  virtual void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert);

  /**
   * If an external source changes values in the costmap,
   * it should call this method with the area that it changed
   * to ensure that the costmap includes this region as well.
   * @param mx0 Minimum x value of the bounding box
   * @param my0 Minimum y value of the bounding box
   * @param mx1 Maximum x value of the bounding box
   * @param my1 Maximum y value of the bounding box
   */

  /**
 * 如果外部源更改了成本地图中的值，它应该调用这个方法，并提供它更改的区域，以确保成本地图也包括这个区域。
 * @param mx0 边界框的最小x值
 * @param my0 边界框的最小y值
 * @param mx1 边界框的最大x值
 * @param my1 边界框的最大y值
 */
  void addExtraBounds(double mx0, double my0, double mx1, double my1);

protected:
  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * TrueOverwrite means every value from this layer
   * is written into the master grid.
   */

  /*
   * 使用本层的值，在指定的边界框内更新 master_grid。
   *
   * TrueOverwrite 意味着从本层的每个值都被写入到 master_grid 中。
   */
  void updateWithTrueOverwrite(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * Overwrite means every valid value from this layer
   * is written into the master grid (does not copy NO_INFORMATION)
   */
  void updateWithOverwrite(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * Sets the new value to the maximum of the master_grid's value
   * and this layer's value. If the master value is NO_INFORMATION,
   * it is overwritten. If the layer's value is NO_INFORMATION,
   * the master value does not change.
   */


  /*
   * 在指定的边界框内，使用本层的值更新 master_grid。
   *
   * 将新值设置为 master_grid 的值和本层的值中的较大值。如果 master_grid 的值是 NO_INFORMATION（无信息），则会被覆盖。
   * 如果本层的值是 NO_INFORMATION，master_grid 的值则不变。
   */
  void updateWithMax(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j);

  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * Sets the new value to the sum of the master grid's value
   * and this layer's value. If the master value is NO_INFORMATION,
   * it is overwritten with the layer's value. If the layer's value
   * is NO_INFORMATION, then the master value does not change.
   *
   * If the sum value is larger than INSCRIBED_INFLATED_OBSTACLE,
   * the master value is set to (INSCRIBED_INFLATED_OBSTACLE - 1).
   */
  void updateWithAddition(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j);

  /**
   * Updates the bounding box specified in the parameters to include
   * the location (x,y)
   *
   * @param x x-coordinate to include
   * @param y y-coordinate to include
   * @param min_x bounding box
   * @param min_y bounding box
   * @param max_x bounding box
   * @param max_y bounding box
   */
  void touch(double x, double y, double * min_x, double * min_y, double * max_x, double * max_y);

  /*
   * Updates the bounding box specified in the parameters
   * to include the bounding box from the addExtraBounds
   * call. If addExtraBounds was not called, the method will do nothing.
   *
   * Should be called at the beginning of the updateBounds method
   *
   * @param min_x bounding box (input and output)
   * @param min_y bounding box (input and output)
   * @param max_x bounding box (input and output)
   * @param max_y bounding box (input and output)
   */

  /*
   * 更新在参数中指定的边界框，以包含从 addExtraBounds 调用中的边界框。
   * 如果没有调用 addExtraBounds，则该方法将不执行任何操作。
   *
   * 应该在 updateBounds 方法的开始时调用
   *
   * @param min_x 边界框（输入和输出）
   * @param min_y 边界框（输入和输出）
   * @param max_x 边界框（输入和输出）
   * @param max_y 边界框（输入和输出）
   */
  void useExtraBounds(double * min_x, double * min_y, double * max_x, double * max_y);
  bool has_extra_bounds_;

private:
  double extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_;
};

}  // namespace nav2_costmap_2d
#endif  // NAV2_COSTMAP_2D__COSTMAP_LAYER_HPP_

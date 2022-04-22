/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 * Author: Stephan Kurzawe
 *********************************************************************/
#ifndef COSTMAP_PROHIBITION_LAYER_H_
#define COSTMAP_PROHIBITION_LAYER_H_

#include <jsoncpp/json/json.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <tf/transform_datatypes.h>
#include <mutex>
#include <geometry_msgs/PoseArray.h>
#include <stdlib.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_prohibition_layer/CostmapProhibitionLayerConfig.h>
#include <dynamic_reconfigure/server.h>

#include <unordered_map>

namespace costmap_prohibition_layer_namespace
{

// point with integer coordinates
struct PointInt
{
    int x;
    int y;
};

class CostmapProhibitionLayer : public costmap_2d::Layer
{
public:

  /**
   * default constructor
   */
  CostmapProhibitionLayer();

  /**
   * destructor
   */
  virtual ~CostmapProhibitionLayer();
  /**
   * function which get called at initializing the costmap
   * define the reconfige callback, get the reoslution
   * and read the prohibitions from the ros-parameter server
   */
  virtual void onInitialize();

  /**
   * 这被 LayeredCostmap 调用来轮询这个插件
   * 关于它需要更新多少成本图。
   * 每一层都可以增加这个边界的大小。
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double *min_x, double *min_y, double *max_x, double *max_y);

  /**
   * 在覆盖成本图的每个成本更新过程中调用的函数。 之前读取的cost将被填补
   */
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                           int max_i, int max_j);

private:
    ros::Subscriber wall_update_sub;
    void updateWallCallback(const std_msgs::Int32& msg);

  /**
   * 计算当前点和多边形集的世界坐标范围。结果存储在类成员 _min_x、_min_y、_max_x 和 _max_y 中。
   */
  void computeMapBounds();

  /**
   * 在 Costmap2D 中为多边形set cost（多边形可能位于边界之外）
   *
   * @param master_grid    对 Costmap2D 对象的引用
   * @param polygon        由点向量定义的多边形多边形（在世界坐标中）
   * @param cost           要设置的cost值 (0,255)
   * @param min_i          水平地图索引/坐标的最小界限
   * @param min_j          垂直地图索引/坐标的最小界限
   * @param max_i          水平地图索引/坐标上的最大界限
   * @param max_j          垂直地图索引/坐标上的最大界限
   * @param fill_polygon   如果为真，多边形内部的cost也将被设置
   */
  void setPolygonCost(costmap_2d::Costmap2D &master_grid, const std::vector<geometry_msgs::Point>& polygon,
                      unsigned char cost, int min_i, int min_j, int max_i, int max_j, bool fill_polygon);

  /**
   * 将多边形（在地图坐标中）转换为地图中的一组单元格
   *
   * @remarks 该方法主要基于 Costmap_2D::convex Fill Cells() 但考虑了一个自行实现的 polygonOutlineCells() 方法并允许负地图坐标
   *
   * @param polygon             由地图坐标向量定义的多边形
   * @param[out] polygon_cells  地图坐标中的新单元格被推回此容器
   * @param fill                如果为真，多边形的内部也将被考虑
   */
  void rasterizePolygon(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells, bool fill);

  /**
   * 根据地图单元提取多边形的边界
   *
   * @remarks 此方法基于 Costmap2D::polygonOutlineCells() 但考虑了自行实现的光线跟踪算法并允许负地图坐标
   *
   * @param polygon             由地图坐标向量定义的多边形
   * @param[out] polygon_cells  地图坐标中的新单元格被推回此容器  container
   */
  void polygonOutlineCells(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells,bool fill);

  /**
   * Rasterize 两个地图坐标之间的线到一组单元格中
   *
   * @remarks 由于 Costmap2D::raytraceLine() 基于 size_x 并且我们想要栅格化可能也位于地图边界之外的多边形，
   *        我们提供了一个基于此处提供的整数版本的修改后的光线跟踪实现
   *        （也是 Bresenham）：http://playtechs .blogspot.de/2007/03/raytracing-on-grid.html
   *
   * @param x0          线起点 x 坐标（地图框）
   * @param y0          线起点 y 坐标（地图框）
   * @param x1          线终点 x 坐标（地图框）
   * @param y1          线终点 y 坐标（地图框）
   * @param[out] cells  地图坐标中的新单元格被推回此容器
   */
  void raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt>& cells);

  /**
   * 从 json 格式中读取禁止区域
   *
   * @return bool       如果解析成功则为 true 否则为 false
   *
    */
  bool parseProhibitionListFromJson();

  dynamic_reconfigure::Server<CostmapProhibitionLayerConfig>* _dsrv;            //!< costmap 的动态重新配置服务器
  std::mutex _data_mutex;                                                       //!< 用于访问 _prohibition_points 和 _prohibition_polygons 的互斥锁
  double _costmap_resolution;                                                   //!< 覆盖costmap的分辨率以创建两点中最细的线
  //bool _fill_polygons;                                                          //!< 是否 多边形内部的所有单元格也被标记为障碍物
  //std::vector<geometry_msgs::Point> _prohibition_points;                        //!< vector 保存源坐标中的单个点
  //std::vector<std::vector<geometry_msgs::Point>> _prohibition_polygons;         //!< vector 保存源坐标 多边形（包括线）
  double _min_x, _min_y, _max_x, _max_y;                                        //!< cached 地图边界
  std::string  config = "~/map/spiritMaps.json";
  Json::Value shapeAreas;
};
}
#endif
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
#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <costmap_prohibition_layer/costmap_prohibition_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_prohibition_layer_namespace::CostmapProhibitionLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace costmap_prohibition_layer_namespace
{

CostmapProhibitionLayer::CostmapProhibitionLayer()
{
}

CostmapProhibitionLayer::~CostmapProhibitionLayer()
{
}

void CostmapProhibitionLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);

  // get a pointer to the layered costmap and save resolution
  costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
  _costmap_resolution = costmap->getResolution();

    // ROS_ERROR("catalpa");
    wall_update_sub = nh.subscribe("/wall_update", 1, &CostmapProhibitionLayer::updateWallCallback, this);
  // set initial bounds
  _min_x = _min_y = _max_x = _max_y = 0;
  if (!parseProhibitionListFromJson())
    ROS_ERROR_STREAM("Reading prohibition areas from failed!");
  // 计算当前禁止区域集的地图边界。
  computeMapBounds();

  ROS_INFO("CostmapProhibitionLayer initialized.");
}
    void CostmapProhibitionLayer::updateWallCallback(const std_msgs::Int32 &msg) {

  // set initial bounds
  if (!parseProhibitionListFromJson())
    ROS_ERROR_STREAM("Reading prohibition areas from failed!");
  // 计算当前禁止区域集的地图边界。
  computeMapBounds();
        ROS_INFO("CostmapProhibitionLayer catalpa has update .");
}

void CostmapProhibitionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                           double *min_x, double *min_y, double *max_x, double *max_y)
{
    std::lock_guard<std::mutex> l(_data_mutex);

    if (shapeAreas.empty())
        return;
    *min_x = std::min(*min_x, _min_x);
    *min_y = std::min(*min_y, _min_y);
    *max_x = std::max(*max_x, _max_x);
    *max_y = std::max(*max_y, _max_y);

}

void CostmapProhibitionLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (shapeAreas.size()<=0)
    return;

  std::lock_guard<std::mutex> l(_data_mutex);

  //
  geometry_msgs::Point point;
  for (Json::Value shape: shapeAreas)
  {
    std::string shae,dui;
    dui ="lines";
    shae =shape["type"].asString();
    //  ROS_ERROR("catalpa type11111111 ");
    //  std::cout<<shae<<std::endl;
    // if(std::strcmp(dui.c_str(),shae.c_str())==0){
    if(dui==shae){
      // ROS_ERROR("catalpa line size: %d",shape["lines"].size());
      if(shape["lines"].size()==1){
        point.x=shape["lines"][0]["x"].asDouble();
        point.y=shape["lines"][0]["y"].asDouble();
        unsigned int mx;
        unsigned int my;
        if (master_grid.worldToMap(point.x, point.y, mx, my)){
          //设置单点cost
          master_grid.setCost(mx, my, LETHAL_OBSTACLE);
        }
        //
      }
      else{
         std::vector<PointInt> map_polygon;
         PointInt loc;
         //遍历寻找
        for (Json::Value po : shape["lines"]){
          point.x =po["x"].asDouble();
          point.y =po["y"].asDouble();
          //转换cos坐标系
          master_grid.worldToMapNoBounds(point.x, point.y, loc.x, loc.y);
          map_polygon.push_back(loc);
        }
        if(shape["lines"].size()==2){//凑3个
          map_polygon.push_back(loc);
        }
        std::vector<PointInt> polygon_cells;
        //获取填充多边形的单元格
        rasterizePolygon(map_polygon, polygon_cells, shape["closed"].asBool());

        // set the cost of those cells
        for (unsigned int i = 0; i < polygon_cells.size(); ++i)
        {
            int mx = polygon_cells[i].x;
            int my = polygon_cells[i].y;
            // check if point is outside bounds
            if (mx < min_i || mx >= max_i)
                continue;
            if (my < min_j || my >= max_j)
                continue;
            master_grid.setCost(mx, my, LETHAL_OBSTACLE);
        }
      }
    }
  }
}

void CostmapProhibitionLayer::computeMapBounds()
{
  std::lock_guard<std::mutex> l(_data_mutex);

  // reset bounds
  // _min_x = _min_y = _max_x = _max_y = 0;

  // iterate polygons
  for (int i = 0; i < shapeAreas.size(); ++i)
  {
    for (int j=0; j < shapeAreas[i]["lines"].size(); ++j)
    {
      double px = shapeAreas[i]["lines"][j]["x"].asDouble();
      double py = shapeAreas[i]["lines"][j]["y"].asDouble();
      _min_x = std::min(px, _min_x);
      _min_y = std::min(py, _min_y);
      _max_x = std::max(px, _max_x);
      _max_y = std::max(py, _max_y);
    }
  }
}


void CostmapProhibitionLayer::polygonOutlineCells(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells,bool fill)
  {
     for (unsigned int i = 0; i < polygon.size() - 1; ++i)
     {
       raytrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
     }
     if (!polygon.empty()&&fill)
     {
       unsigned int last_index = polygon.size() - 1;
       // 我们还需要通过从最后一个点到第一个点来关闭多边形
       raytrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
     }
  }

void CostmapProhibitionLayer::raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt>& cells)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    PointInt pt;
    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n)
    {
        cells.push_back(pt);

        if (error > 0)
        {
            pt.x += x_inc;
            error -= dy;
        }
        else
        {
            pt.y += y_inc;
            error += dx;
        }
    }
}


void CostmapProhibitionLayer::rasterizePolygon(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells, bool fill)
{
    // this implementation is a slighly modified version of Costmap2D::convexFillCells(...)

    //we need a minimum polygon of a traingle
    if(polygon.size() < 3)
        return;

    //获取构成多边形轮廓的单元格
    polygonOutlineCells(polygon, polygon_cells,fill);

    if (!fill)
        return;

    //快速冒泡排序按 x 对点进行排序
    PointInt swap;
    unsigned int i = 0;
    while(i < polygon_cells.size() - 1)
    {
        if(polygon_cells[i].x > polygon_cells[i + 1].x)
        {
            swap = polygon_cells[i];
            polygon_cells[i] = polygon_cells[i + 1];
            polygon_cells[i + 1] = swap;

            if(i > 0)
            --i;
        }
        else
            ++i;
        }

        i = 0;
        PointInt min_pt;
        PointInt max_pt;
        int min_x = polygon_cells[0].x;
        int max_x = polygon_cells[(int)polygon_cells.size() -1].x;

        //walk through each column and mark cells inside the polygon
        for(int x = min_x; x <= max_x; ++x)
        {
            if(i >= (int)polygon_cells.size() - 1)
                break;

            if(polygon_cells[i].y < polygon_cells[i + 1].y)
            {
                min_pt = polygon_cells[i];
                max_pt = polygon_cells[i + 1];
            }
            else
            {
                min_pt = polygon_cells[i + 1];
                max_pt = polygon_cells[i];
            }

            i += 2;
            while(i < polygon_cells.size() && polygon_cells[i].x == x)
            {
                if(polygon_cells[i].y < min_pt.y)
                    min_pt = polygon_cells[i];
                else if(polygon_cells[i].y > max_pt.y)
                    max_pt = polygon_cells[i];
                ++i;
            }

            PointInt pt;
            //loop though cells in the column
            for(int y = min_pt.y; y < max_pt.y; ++y)
            {
                pt.x = x;
                pt.y = y;
                polygon_cells.push_back(pt);
            }
        }
  }

// 从文件加载禁止位置
bool CostmapProhibitionLayer::parseProhibitionListFromJson()
{

/*
 */
std::lock_guard<std::mutex> l(_data_mutex);
shapeAreas.clear();
Json::Value root;
Json::Reader reader;
std::ifstream ifs(config);//open file example.json
if(!ifs.is_open())
{
    ROS_ERROR_STREAM("costmap prohibition catalpa '"<<config<<"' not find");
return false;
}
if(!reader.parse(ifs, root)){
   // fail to parse
    ROS_ERROR_STREAM("costmap prohibition catalpa fail to parse of '"<<config<<"'.");
   return false;
}
else{
   // success
   std::int64_t deffloor =0;
   deffloor =root["defaultFloor"].asInt64();
   Json::Value floors = root["floors"];
   for(Json::Value floor : floors){
     if(floor["floorId"].asInt64() == deffloor){
      // ROS_INFO_STREAM("floorName: "<<floor["floorName"].asString());
      Json::Value localMaps = floor["localMap"];
      std::int64_t dfmap = localMaps["defaultmap"].asInt64();
      Json::Value maps = localMaps["maps"];
      for(Json::Value map :maps){
        if(map["mapId"].asInt64() == dfmap){
          // ROS_INFO_STREAM("mapName: "<<map["mapName"].asString());
          shapeAreas = map["shapeAreas"];
          // ROS_ERROR("catalp1 shapeAreas size: %d",shapeAreas.size());
          break;
        }
      }
      break;
     }
   }

}
    ifs.close();

  return true;
}

}  // end namespace

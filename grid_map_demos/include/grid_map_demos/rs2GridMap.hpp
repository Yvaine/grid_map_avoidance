/*
 * rs2GridMap.h
 *
 * Create on: Feb 3, 2016
 *   Author: Leng Jing
 *   descriptor: realsense structure to GridMap object
 *
 */
#pragma once

// realsense
#include "grid_map_demos/librealsense/rs.hpp"
//#include "grid_map_demos/example.hpp"

//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <grid_map/grid_map.hpp>

#include <string>

namespace grid_map_demos {

class rs2GridMap
{
    public:

       rs2GridMap();

       virtual ~rs2GridMap();

       rs::device * dev;
       grid_map::GridMap gridMap;
       //! Length of the grid map in x direction.
       double mapLengthX;
       double mapLengthY;

       //! Resolution of the grid map.
       double resolution;
       //! Range of the height values.
       double minHeight;
       double maxHeight;

       int pointCloudSize;
       std::vector<rs::float3>cloudPointsRS;
       rs::float3 *pcloudPointsRS;


        void initDevice();
        void initMap();
        void getPointCloud();
        void PointCloud2GirdMap();
};
}

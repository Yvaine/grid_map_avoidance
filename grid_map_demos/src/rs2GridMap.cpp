/*
 * rs2GridMap.cpp
 *
 * Create on: Feb 3, 2016
 *   Author: Leng Jing
 *   Institute : Nienbot
 *   descriptor: realsense structure to GridMap object
 *
 */

#include "grid_map_demos/rs2GridMap.hpp"

#include <chrono>
#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>


using namespace std;
using namespace ros;
using namespace grid_map;

namespace grid_map_demos{

    rs2GridMap::rs2GridMap():dev(NULL),pointCloudSize(0),
                             pcloudPointsRS(NULL),resolution(0.1),
                             mapLengthX(3), mapLengthY(3), gridMap({"elevation"})
    {
        initMap();
        initDevice();
    }

    rs2GridMap::~rs2GridMap(){};

    void rs2GridMap::initDevice()
    {
        rs::log_to_console(rs::log_severity::warn);
        rs::context ctx;
        if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
        dev = ctx.get_device(0);

        dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
        dev->enable_stream(rs::stream::color, rs::preset::best_quality);
        dev->start();
        ROS_INFO("Start RealSense");
#if 0
        std::ostringstream ss;
        ss << "CPP Point Cloud Example (" << dev->get_name() << ")";
#endif
    }

    void rs2GridMap::initMap()
    {
        //GridMap gridMap({"elevation"});
        //const double lengthX = resolution * cellnumX;
        //const double lengthY = resolution * cellnumY;
        Length length(mapLengthX, mapLengthY);
        Position MapPosition = Position(0,0);
        gridMap.setGeometry(length, resolution, MapPosition);
        gridMap.setFrameId("map");
        ROS_INFO("Init Map");
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                gridMap.getLength().x(), gridMap.getLength().y(),
                gridMap.getSize()(0), gridMap.getSize()(1));
    }

    void rs2GridMap::getPointCloud()
    {
        cloudPointsRS.clear();
        ROS_INFO("getPointCloud0");
        const float depth_scale = dev->get_depth_scale();
        ROS_INFO("getPointCloud1");
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
        ROS_INFO("getPointCloud2");
        //const rs::extrinsics extrin = dev->get_extrinsics(rs::stream::depth, tex_stream);
        const rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        ROS_INFO("getPointCloud4");
        pointCloudSize = 0;
        ROS_INFO("getPointCloud");
#if 0
        auto pcloudPointsRS_ = reinterpret_cast<const rs::float3 *>(dev->get_frame_data(rs::stream::points));
        pcloudPointsRS = (rs::float3 *)pcloudPointsRS_;
        rs::float3 *pIndex = pcloudPointsRS;
        while(pIndex != NULL)
        {
            pointCloudSize++;
            pIndex++;
        }
#else
    for(int dy=0; dy<depth_intrin.height; ++dy)
    {
        for(int dx=0; dx<depth_intrin.width; ++dx)
        {
            uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
            float depth_in_meters = depth_value * depth_scale;
            if(depth_value = 0) continue;

            rs::float2 depth_pixel = {(float)dx, (float)dy};
            rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
            pointCloudSize++;
            cloudPointsRS.push_back(depth_point);
        }
    }
#endif
    }


    void rs2GridMap::PointCloud2GirdMap()
    {
        GridMapIterator itGlobal(gridMap);
        for(; !itGlobal.isPastEnd(); ++itGlobal)
        {
            gridMap.at("elevation", *itGlobal) = 0;
        }

        for(int points_num = 0; points_num < pointCloudSize; points_num++)
        {
            Position position;
            position.x() = pcloudPointsRS->x;
            position.y() = pcloudPointsRS->z;
            Index positionIndex;
            gridMap.getIndex(position, positionIndex);
            Size size = Length(1, 1).cast<int>();
            SubmapIterator it(gridMap, positionIndex, size);
            for(; !it.isPastEnd(); ++it)
            {
                gridMap.at("elevation", *it) = pcloudPointsRS->y;

            }
            pcloudPointsRS++;

        }
    }








}







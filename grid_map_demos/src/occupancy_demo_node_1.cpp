#include <ros/ros.h>
#include <grid_map/grid_map.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

//#include "grid_map_demos/rs2GridMap.hpp"
using namespace grid_map;
using namespace std;
using namespace ros;

#include <grid_map_demos/librealsense/rs.hpp>
#include <cstdio>


#include <chrono>
#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>
int main(int argc, char** argv)try
{

    grid_map::GridMap gridMap({"elevation"});
    //initMap
   int mapLengthX = 20;
    int mapLengthY = 20;
    double resolution = 0.1;

    Length length(mapLengthX, mapLengthY);
    Position MapPosition = Position(0,0);
    gridMap.setGeometry(length, resolution, MapPosition);
    gridMap.setFrameId("map");
    ROS_INFO("Init Map");
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
            gridMap.getLength().x(), gridMap.getLength().y(),
            gridMap.getSize()(0), gridMap.getSize()(1));

    //initDevice
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;

    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    // Configure depth and color to run with the device's preferred settings
    dev->enable_stream(rs::stream::depth, 320, 240, rs::format::z16, 60);
    dev->enable_stream(rs::stream::color, 320, 240, rs::format::rgb8, 60);
    dev->enable_stream(rs::stream::infrared, 320, 240, rs::format::y8, 60);
    try { dev->enable_stream(rs::stream::infrared2, 320, 240, rs::format::y8, 60); }
    catch(...) { printf("Device does not provide infrared2 stream.\n"); }
    dev->start();

    // ROS
    ros::init(argc, argv, "grid_map_occupancy_demo");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    // Work with grid map in a loop.
    ros::Rate rate(30.0);
    timeval starttime, endtime;
    int time = 0;
    //int FPS = 0;
    std::vector<rs::float3>cloudPointsRS;
    cloudPointsRS.reserve(5000);
    while (nh.ok()) {
        //while(1)
#if 0
        if(time > 1000)
        {
            ROS_INFO("FPS is %d", FPS);
            FPS = 0;
             time = 0;
        }
#endif
        gettimeofday(&starttime,0);
        dev->wait_for_frames();
        //ROS_INFO("wait for frames");
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
        //const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);

        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        //rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
        //rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
        float depth_scale = dev->get_depth_scale();
        //ROS_INFO("wait for frames1");

        // Get PointCloud
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
        int pointCloudSize = 0;
        cloudPointsRS.clear();

        for(int dy=0; dy<depth_intrin.height; ++dy)
        {
            for(int dx=0; dx<depth_intrin.width; ++dx)
            {
                uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
                float depth_in_meters = depth_value * depth_scale;
                if(depth_value == 0) continue;

                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                pointCloudSize++;
                cloudPointsRS.push_back(depth_point);
            }
        }

        //ROS_INFO("pointCloudSize : %d", pointCloudSize);
#endif
        // pointCloud2GridMap
        GridMapIterator itGlobal(gridMap);
        gridMap.clearAll();
#if 0
        for(; !itGlobal.isPastEnd(); ++itGlobal)
        {
            gridMap.at("elevation", *itGlobal) = 0;
        }
#endif
        for(int points_num = 0; points_num < pointCloudSize; points_num++)
        {
            Position position;
            rs::float3 pcloudPointsRS = cloudPointsRS.at(points_num);
            position.x() = pcloudPointsRS.x;
            position.y() = pcloudPointsRS.z;
            Index positionIndex;
            gridMap.getIndex(position, positionIndex);
#if 0
            Size size = Length(1, 1).cast<int>();
            SubmapIterator it(gridMap, positionIndex, size);
            for(; !it.isPastEnd(); ++it)
            {
                //gridMap.at("elevation", *it) = pcloudPointsRS.y;
                if(pcloudPointsRS.y < -1 && pcloudPointsRS.y > 1)
                    gridMap.at("elevation", *it) = 0;
                gridMap.at("elevation", *it) = 1;

            }
#else
            if(pcloudPointsRS.y > -1 && pcloudPointsRS.y < 1)
            gridMap.at("elevation", positionIndex) = 1;

#endif
        }
        gettimeofday(&endtime,0);
        time = (1000000*(endtime.tv_sec - starttime.tv_sec) + endtime.tv_usec - starttime.tv_usec) / 1000;
        ROS_INFO("time is %d", time);
        //FPS++;
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(gridMap, message);
        publisher.publish(message);
        //ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

        // Wait for next cycle.
        rate.sleep();
        //

    }
#if 0
    grid_map_demos::rs2GridMap rsMap;
    // Initialize node and publisher.
    ros::init(argc, argv, "grid_map_occupancy_demo");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    // Work with grid map in a loop.
    ros::Rate rate(30.0);
    //while (nh.ok()) {
    while(1){

        // Add data to grid map.
        ros::Time time = ros::Time::now();


        ROS_INFO("wait for frames");
        rsMap.dev->wait_for_frames();
        ROS_INFO("wait for frames1");
        rsMap.getPointCloud();
        //rsMap.PointCloud2GirdMap();
        // Publish grid map.
    rsMap.gridMap.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(rsMap.gridMap, message);
    publisher.publish(message);
    //ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    // Wait for next cycle.
    rate.sleep();
  }
  return 0;
#endif
    return EXIT_SUCCESS;
}

catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}

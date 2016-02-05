#include <ros/ros.h>
#include <grid_map/grid_map.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

#include "grid_map_demos/rs2GridMap.hpp"
using namespace grid_map;

int main(int argc, char** argv)try
{
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
}

catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}

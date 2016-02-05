#include <ros/ros.h>
#include <grid_map/grid_map.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

using namespace grid_map;

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_simple_demo");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // Create grid map.
  GridMap map({"elevation"});
  map.setFrameId("map");
  Eigen::Array2d mapLength = Length(6, 2);
  float resolution = 1;
  map.setGeometry(mapLength, resolution);
  //Eigen::Vector2d vectorToOrigin = (0.5 * mapLength).matrix();
  //ROS_INFO("vectorToOrigin is %.2lf x %.2lf y", vectorToOrigin[0], vectorToOrigin[1]);
  //Eigen::Vector2d vectorToFirstCell = (vectorToOrigin.array() - 0.5 * resolution).matrix();
  //ROS_INFO("vectorToFirstCell is %.2lf x %.2lf y", vectorToFirstCell[0], vectorToFirstCell[1]);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  // Work with grid map in a loop.
  ros::Rate rate(30.0);
  while (nh.ok()) {

    // Add data to grid map.
    ros::Time time = ros::Time::now();
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      //map.at("elevation", *it) = 1.0;
      map.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
    }

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    // Wait for next cycle.
    rate.sleep();
  }

  return 0;
}

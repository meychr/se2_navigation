/*
 * se2_planner_node.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include "se2_grid_map_test/GridMapTest.hpp"

int main(int argc, char** argv) {
  using namespace se2_planning;

  ros::init(argc, argv, "se2_grid_map_test_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  GridMapTest test = GridMapTest(nh);
  test.initialize();

  // rostopic pub /se2_grid_map_test_node/obstacle geometry_msgs/Point "{x: 1.0, y: 0.0, z: 0.0}" -1
  // to trigger new map

  ros::spin();

  return 0;
}

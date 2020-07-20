/*
 * se2_planner_node.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include "se2_planning/GridMapLazyStateValidator.hpp"
#include "se2_planning_ros/loaders.hpp"

int main(int argc, char** argv) {
  using namespace se2_planning;

  ros::init(argc, argv, "se2_planner_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  // Load planner parameters
  std::string filename = nh->param<std::string>("/ompl_planner_ros/parameter_path", "ompl_rs_planner_ros/nav_msgs_path");
  const auto plannerParameters = loadOmplReedsSheppPlannerParameters(filename);
  const auto plannerRosParameters = loadOmplReedsSheppPlannerRosParameters(filename);
  auto planner = std::make_shared<OmplReedsSheppPlanner>();
  planner->setParameters(plannerParameters);

  // Publish grid map for debugging
  ros::Publisher publisher = nh->advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // Grid map update has to be thread safe?
  // Create test grid map
  std::string layerName = "traversability";
  grid_map::GridMap gridMap;
  gridMap.setFrameId("world");
  gridMap.setGeometry(grid_map::Length(20.0, 20.0), 0.1, grid_map::Position(0, 0));  // adjust planner parameters as well!
  gridMap.add(layerName, 0.0);

  // Add obstacles to traversability layer
  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    gridMap.getPosition(*iterator, position);
    if (position.x() < 7 && position.x() > 5 && fabs(position.y()) < 2) {
      gridMap.at(layerName, *iterator) = 1.0;  // obstacles
    }
  }

  // Publish grid map for debugging
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(gridMap, message);
  publisher.publish(message);

  // Set grid map validator with static map
  planner->setStateValidator(
      se2_planning::createGridMapLazyStateValidator(gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), layerName));

  // Setup ROS interface and start node
  se2_planning::OmplReedsSheppPlannerRos plannerRos(nh);
  plannerRos.setPlanningStrategy(planner);
  plannerRos.setParameters(plannerRosParameters);
  plannerRos.initialize();

  ros::spin();

  return 0;
}

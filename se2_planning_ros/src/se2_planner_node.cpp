/*
 * se2_planner_node.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include "se2_planning/OmplReedsSheppPlanner.hpp"
#include "se2_planning_ros/GridMapLazyStateValidatorRos.hpp"
#include "se2_planning_ros/loaders.hpp"

using namespace se2_planning;

auto planner = std::make_shared<OmplReedsSheppPlanner>();

void gridMapCallback(const grid_map_msgs::GridMap& msg) {
  // Update planner bounds when new map is published, placed here due to class structure
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);
  // Grid map is symmetric around position
  grid_map::Position mapPosition;
  mapPosition = map.getPosition();
  grid_map::Length mapLength;
  mapLength = map.getLength();
  ompl::base::RealVectorBounds bounds(2);
  bounds.low[0] = mapPosition.x() - mapLength.x() / 2.0;
  bounds.low[1] = mapPosition.y() - mapLength.y() / 2.0;
  bounds.high[0] = mapPosition.x() + mapLength.x() / 2.0;
  bounds.high[1] = mapPosition.y() + mapLength.y() / 2.0;
  ROS_INFO_STREAM("pos: " << mapPosition.x() << ", " << mapPosition.y() << ", length: " << mapLength.x() << ", " << mapLength.y());
  planner->updateStateSpaceBounds(bounds);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "se2_planner_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  // Load planner parameters
  std::string filename = nh->param<std::string>("/ompl_planner_ros/parameter_path", "ompl_rs_planner_ros/nav_msgs_path");
  const auto plannerParameters = loadOmplReedsSheppPlannerParameters(filename);
  const auto plannerRosParameters = loadOmplReedsSheppPlannerRosParameters(filename);
  const auto stateValidatorRosParameters = loadGridMapLazyStateValidatorRosParameters(filename);
  planner->setParameters(plannerParameters);

  // Create initial grid map
  grid_map::GridMap gridMap;
  gridMap.setFrameId(stateValidatorRosParameters.gridMapFrame_);
  gridMap.setGeometry(grid_map::Length(20.0, 20.0), 0.1, grid_map::Position(0, 0));  // adjust planner parameters as well!
  gridMap.add(stateValidatorRosParameters.gridMapObstacleLayerName_, 0.0);

  // Set grid map state validator
  // TODO adapt footprint, move to param?
  planner->setStateValidator(se2_planning::createGridMapLazyStateValidatorRos(nh, stateValidatorRosParameters, gridMap,
                                                                              se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5),
                                                                              stateValidatorRosParameters.gridMapObstacleLayerName_));

  // Setup ROS interface and start node
  se2_planning::OmplReedsSheppPlannerRos plannerRos(nh);
  plannerRos.setPlanningStrategy(planner);
  plannerRos.setParameters(plannerRosParameters);
  plannerRos.initialize();

  // Callback to update OMPL planner bounds when map changes
  ros::Subscriber mapSub = nh->subscribe(stateValidatorRosParameters.gridMapMsgTopic_, 1, gridMapCallback);

  ros::spin();

  return 0;
}

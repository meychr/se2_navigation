/*
 * GridMapLazyStateValidatorRos.cpp
 *
 *  Created on: Jul 20, 2020
 *      Author: meyerc
 */

#include "se2_planning_ros/GridMapLazyStateValidatorRos.hpp"

#include <thread>

namespace se2_planning {

GridMapLazyStateValidatorRos::GridMapLazyStateValidatorRos(ros::NodeHandlePtr nh) : BASE(), nh_(nh), newMapAvailable_(false) {}

void GridMapLazyStateValidatorRos::setParameters(const GridMapLazyStateValidatorRosParameters& parameters) {
  parameters_ = parameters;
}

void GridMapLazyStateValidatorRos::initialize() {
  BASE::initialize();
  initRos();
  initializeMap();
}

void GridMapLazyStateValidatorRos::initializeMap() {
  std::string mapFrameId_ = parameters_.gridMapFrame_;
  double mapLengthX = 20.0;
  double mapLengthY = 20.0;
  map_.setFrameId(mapFrameId_);
  map_.setTimestamp(ros::Time::now().toNSec());
  map_.setGeometry(grid_map::Length(mapLengthX, mapLengthY), parameters_.gridMapResolution_,
                   grid_map::Position(0, 0));  // adjust planner parameters as well!
  map_.add(obstacleLayerName_, 0.0);
  setGridMap(map_);
  publishMap();
}

void GridMapLazyStateValidatorRos::mapCb(const grid_map_msgs::GridMap& msg) {
  if (isLocked()) {
    ROS_INFO_STREAM("Planner is running, grid map for state validator can not be updated!");
    return;
  } else {
    grid_map::GridMap newMap;
    grid_map::GridMapRosConverter::fromMessage(msg, newMap);

    isGridMapInitialized_ = false;

    if (newMap.exists(obstacleLayerName_)) {
      // Convert resolution
      map_.setGeometry(newMap.getLength(), parameters_.gridMapResolution_, newMap.getPosition());
      map_.addDataFrom(newMap, true, true, true);
      setGridMap(map_);
      publishMap();
      newMapAvailable_ = true;
      ROS_INFO_STREAM("GlobalMap: Convert initial map resolution from " << newMap.getResolution() << " to "
                                                                        << parameters_.gridMapResolution_ << ".");
    } else {
      ROS_ERROR("GlobalMap: No traversability layer found to load!");
    }

    isGridMapInitialized_ = true;
  }
}

void GridMapLazyStateValidatorRos::initRos() {
  mapPublisher_ = nh_->advertise<grid_map_msgs::GridMap>("map_debug", 1);
  mapSubscriber_ = nh_->subscribe(parameters_.gridMapMsgTopic_, 1, &GridMapLazyStateValidatorRos::mapCb, this);
}

void GridMapLazyStateValidatorRos::publishMap() const {
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(map_, msg);
  mapPublisher_.publish(msg);
}

std::unique_ptr<GridMapLazyStateValidatorRos> createGridMapLazyStateValidatorRos(const ros::NodeHandlePtr nh,
                                                                                 const GridMapLazyStateValidatorRosParameters& params,
                                                                                 const grid_map::GridMap& gridMap,
                                                                                 const RobotFootprint& footprint,
                                                                                 const std::string& obstacleLayer) {
  std::unique_ptr<GridMapLazyStateValidatorRos> validator = std::make_unique<GridMapLazyStateValidatorRos>(nh);
  validator->setGridMap(gridMap);
  validator->setObstacleLayerName(obstacleLayer);
  validator->setFootprint(footprint);
  validator->setParameters(params);
  validator->initialize();
  return std::move(validator);
}

} /* namespace se2_planning */
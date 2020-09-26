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
}

void GridMapLazyStateValidatorRos::mapCb(const grid_map_msgs::GridMap& msg) {
  // TODO Also replace using the same gridMapMutex_? Issue that we require to set it in OMPLReedsSheppPlanner which only
  //  has access to StateValidator class => add general state validator mutex?
  if (isLocked()) {
    ROS_INFO_STREAM("Planner is running, grid map for state validator can not be updated!");
    return;
  } else {
    grid_map::GridMap newMap;
    grid_map::GridMapRosConverter::fromMessage(msg, newMap);

    if (newMap.exists(obstacleLayerName_)) {
      WriteLock writeLock(gridMapMutex_);
      setGridMap(newMap);
      newMapAvailable_ = true;
      if (!isGridMapInitialized_) {
        isGridMapInitialized_ = true;
      }
      writeLock.unlock();
      publishMap(getGridMap());
    } else {
      ROS_ERROR("GlobalMap: No traversability layer found to load!");
    }
  }
}

bool GridMapLazyStateValidatorRos::checkPathServer(se2_navigation_msgs::CheckPathSrv::Request& req,
                                                   se2_navigation_msgs::CheckPathSrv::Response& res) {
  if (isGridMapInitialized_) {
    ReadLock readLock(gridMapMutex_);
    bool lastStateInvalid = false;
    res.valid = true;
    res.nextValidPathSegment = 0;
    res.nextValidPoint = 0;
    for (int idx_segment = 0; idx_segment < req.path.segment.size(); idx_segment++) {
      for (int idx_point = 0; idx_point < req.path.segment[idx_segment].points.size(); idx_point++) {
        if (req.path.segment[idx_segment].points[idx_point].orientation.w == 0) {
          ROS_ERROR("Invalid quaternion passed to check path server.");
        }
        SE2state state;
        state = convert(req.path.segment[idx_segment].points[idx_point]);
        // Check if there is an invalid state and set safe path flag to false
        if (!isStateValid(state)) {
          res.valid = false;
          lastStateInvalid = true;
        }
        // If safe path flag is set to false, check if there comes another safe state afterwards
        // TODO: Can happen multiple times with multiple obstacles on the path, how to handle that?
        if (lastStateInvalid && isStateValid(state)) {
          res.nextValidPathSegment = idx_segment;
          res.nextValidPoint = idx_point;
          lastStateInvalid = false;
        }
      }
    }
    readLock.unlock();
    // Check if last state was invalid and then set values such that no valid path exists
    // Leads to same result if only last state is not valid!
    if (lastStateInvalid) {
      res.nextValidPathSegment = req.path.segment.size() + 1;
      res.nextValidPoint = req.path.segment.end()->points.size() + 1;
    }
    return true;
  } else {
    ROS_WARN("Grid map has not been initialized yet. checkPathServer can not check trajectory.");
    return true;
  }
}

void GridMapLazyStateValidatorRos::initRos() {
  // Visualize map used in state validator in rviz
  mapPublisher_ = nh_->advertise<grid_map_msgs::GridMap>(parameters_.gridMapMsgPubTopic_, 1);
  // Input topic for grid map
  mapSubscriber_ = nh_->subscribe(parameters_.gridMapMsgSubTopic_, 1, &GridMapLazyStateValidatorRos::mapCb, this);
  // Check validity of a given path
  checkPathServer_ = nh_->advertiseService(parameters_.checkPathServiceName_, &GridMapLazyStateValidatorRos::checkPathServer, this);
}

void GridMapLazyStateValidatorRos::publishMap(const grid_map::GridMap& map) const {
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(map, msg);
  mapPublisher_.publish(msg);
}

std::unique_ptr<GridMapLazyStateValidatorRos> createGridMapLazyStateValidatorRos(
    const ros::NodeHandlePtr nh, const GridMapLazyStateValidatorRosParameters& params, const grid_map::GridMap& gridMap,
    const RobotFootprint& footprint, const std::string& obstacleLayer, const StateValidityCheckingMethod stateValidityCheckingMethod,
    const double stateValidityCheckingThreshold) {
  std::unique_ptr<GridMapLazyStateValidatorRos> validator = std::make_unique<GridMapLazyStateValidatorRos>(nh);
  validator->setGridMap(gridMap);
  validator->setObstacleLayerName(obstacleLayer);
  validator->setStateValidityCheckingMethod(stateValidityCheckingMethod);
  validator->setStateValidityThreshold(stateValidityCheckingThreshold);
  validator->setFootprint(footprint);
  validator->setParameters(params);
  validator->initialize();
  return std::move(validator);
}

} /* namespace se2_planning */

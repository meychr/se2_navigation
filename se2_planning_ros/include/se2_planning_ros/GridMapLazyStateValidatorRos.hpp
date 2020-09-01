/*
 * GridMapLazyStateValidatorRos.hpp
 *
 *  Created on: Jul 20, 2020
 *      Author: meyerc
 */

#pragma once

#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include "se2_planning/GridMapLazyStateValidator.hpp"

#include <string>

namespace se2_planning {

struct GridMapLazyStateValidatorRosParameters {
  std::string gridMapFrame_ = "map";
  std::string gridMapMsgTopic_ = "state_validator_ros/traversability_map";
  std::string gridMapObstacleLayerName_ = "traversability";  // redundant, already defined by GridMapStateValidator
  double gridMapResolution_ = 0.2;
};

class GridMapLazyStateValidatorRos : public GridMapLazyStateValidator {
  using BASE = GridMapLazyStateValidator;

 public:
  explicit GridMapLazyStateValidatorRos(ros::NodeHandlePtr nh);
  ~GridMapLazyStateValidatorRos() override = default;

  void initialize() override;
  void initializeMap();
  void setParameters(const GridMapLazyStateValidatorRosParameters& parameters);
  void publishMap() const;

 private:
  void initRos();
  void mapCb(const grid_map_msgs::GridMap& msg);

  ros::NodeHandlePtr nh_;
  GridMapLazyStateValidatorRosParameters parameters_;

  //! Grid map data.
  grid_map::GridMap map_;
  ros::Subscriber mapSubscriber_;
  ros::Publisher mapPublisher_;
  bool newMapAvailable_ = false;
};

std::unique_ptr<GridMapLazyStateValidatorRos> createGridMapLazyStateValidatorRos(const ros::NodeHandlePtr nh,
                                                                                 const GridMapLazyStateValidatorRosParameters& params,
                                                                                 const grid_map::GridMap& gridMap,
                                                                                 const RobotFootprint& footprint,
                                                                                 const std::string& obstacleLayer);

} /* namespace se2_planning */
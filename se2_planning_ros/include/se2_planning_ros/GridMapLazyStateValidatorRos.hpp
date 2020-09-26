/*
 * GridMapLazyStateValidatorRos.hpp
 *
 *  Created on: Jul 20, 2020
 *      Author: meyerc
 */

#pragma once

#include <ros/ros.h>

#include <se2_navigation_msgs/CheckPathSrv.h>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include "se2_planning/GridMapLazyStateValidator.hpp"
#include "se2_planning_ros/OmplReedsSheppPlannerRos.hpp"

#include <string>

namespace se2_planning {

typedef boost::unique_lock<boost::shared_mutex> WriteLock;
typedef boost::shared_lock<boost::shared_mutex> ReadLock;

struct GridMapLazyStateValidatorRosParameters {
  std::string gridMapFrame_ = "map";
  std::string gridMapMsgSubTopic_ = "state_validator_ros/traversability_map_in";
  std::string gridMapMsgPubTopic_ = "state_validator_ros/traversability_map_out";
  std::string gridMapObstacleLayerName_ = "traversability";  // redundant, already defined by GridMapStateValidator
  StateValidityCheckingMethod gridMapStateValidityCheckingMethod_ =
      StateValidityCheckingMethod::TRAVERSABILITY;  // redundant, already defined by GridMapStateValidator
  double gridMapStateValidityThreshold_ = 0.5;
  double gridMapResolution_ = 0.2;
  double gridMapLength_ = 20.0;
  double gridMapWidth_ = 20.0;
  double gridMapPositionX_ = 0.0;
  double gridMapPositionY_ = 0.0;
  double gridMapDefaultValue_ = 0.0;
  double robotFootPrintLengthForward_ = 0.75;
  double robotFootPrintLengthBackward_ = 0.75;
  double robotFootPrintWidthLeft_ = 0.5;
  double robotFootPrintWidthRight_ = 0.5;
  std::string checkPathServiceName_ = "check_path";
};

class GridMapLazyStateValidatorRos : public GridMapLazyStateValidator {
  using BASE = GridMapLazyStateValidator;

 public:
  explicit GridMapLazyStateValidatorRos(ros::NodeHandlePtr nh);
  ~GridMapLazyStateValidatorRos() override = default;

  void initialize() override;
  void setParameters(const GridMapLazyStateValidatorRosParameters& parameters);
  void publishMap(const grid_map::GridMap& map) const;

 private:
  void initRos();
  void mapCb(const grid_map_msgs::GridMap& msg);
  bool checkPathServer(se2_navigation_msgs::CheckPathSrv::Request& req, se2_navigation_msgs::CheckPathSrv::Response& res);

  ros::NodeHandlePtr nh_;
  GridMapLazyStateValidatorRosParameters parameters_;

  //! Grid map data.
  ros::Subscriber mapSubscriber_;
  ros::Publisher mapPublisher_;
  bool newMapAvailable_ = false;
  ros::ServiceServer checkPathServer_;
  boost::shared_mutex gridMapMutex_;
};

std::unique_ptr<GridMapLazyStateValidatorRos> createGridMapLazyStateValidatorRos(
    const ros::NodeHandlePtr nh, const GridMapLazyStateValidatorRosParameters& params, const grid_map::GridMap& gridMap,
    const RobotFootprint& footprint, const std::string& obstacleLayer, const StateValidityCheckingMethod stateValidityCheckingMethod,
    const double stateValidityCheckingThreshold);

} /* namespace se2_planning */

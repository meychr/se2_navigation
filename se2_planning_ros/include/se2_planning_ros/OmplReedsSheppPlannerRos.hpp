/*
 * OmplReedsSheppPlannerRos.hpp
 *
 *  Created on: Apr 2, 2020
 *      Author: jelavice
 */

#pragma once

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <string>
#include "se2_navigation_msgs/Path.hpp"
#include "se2_navigation_msgs/RequestPathSrv.h"
#include "se2_planning/OmplReedsSheppPlanner.hpp"
#include "se2_planning_ros/PlannerRos.hpp"
#include "visualization_msgs/Marker.h"

namespace se2_planning {

struct OmplReedsSheppPlannerRosParameters {
  std::string pathFrame_ = "map";
  std::string pathNavMsgTopic_ = "ompl_rs_planner_ros/nav_msgs_path";
  std::string planningSerivceName_ = "ompl_rs_planner_ros/planning_service";
  std::string pathMsgTopic_ = "ompl_rs_planner_ros/path";
  double pathNavMsgResolution_ = 1.0;
  double stateSpaceBoundsMargin_ = 0.5;
};

class OmplReedsSheppPlannerRos : public PlannerRos {
  using BASE = PlannerRos;

 public:
  explicit OmplReedsSheppPlannerRos(ros::NodeHandlePtr nh);
  ~OmplReedsSheppPlannerRos() override = default;

  bool initialize() override;
  bool plan() override;
  void setParameters(const OmplReedsSheppPlannerRosParameters& parameters);
  void publishPath() const final;

 private:
  void initRos();
  void publishPathNavMsgs() const;
  void publishStartGoalMsgs(const ReedsSheppState& start, const ReedsSheppState& goal) const;
  void initializeStateSpaceMarker();
  void publishStateSpaceMarker();
  bool planningService(PlanningService::Request& req, PlanningService::Response& res) override;

  ros::Publisher pathNavMsgsPublisher_;
  ros::Publisher pathPublisher_;
  ros::Publisher startPublisher_;
  ros::Publisher goalPublisher_;
  ros::Publisher stateSpacePublisher_;
  visualization_msgs::Marker stateSpaceMarker_;
  OmplReedsSheppPlannerRosParameters parameters_;
  ros::ServiceServer planningService_;
  int planSeqNumber_ = -1;
  const int reedsSheppStateSpaceDim_ = 2;
};

nav_msgs::Path copyAllPoints(const ReedsSheppPath& path);
geometry_msgs::Pose convert(const ReedsSheppState& state, double z = 0.0);
ReedsSheppState convert(const geometry_msgs::Pose& state);
se2_navigation_msgs::Path convert(const ReedsSheppPath& path);

} /* namespace se2_planning */

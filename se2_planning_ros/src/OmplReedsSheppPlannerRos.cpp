/*
 * OmplReedsSheppPlannerRos.cpp
 *
 *  Created on: Apr 2, 2020
 *      Author: jelavice
 */

#include "se2_planning_ros/OmplReedsSheppPlannerRos.hpp"
#include "se2_planning_ros/common.hpp"

#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <thread>

namespace se2_planning {

OmplReedsSheppPlannerRos::OmplReedsSheppPlannerRos(ros::NodeHandlePtr nh) : BASE(nh) {}

void OmplReedsSheppPlannerRos::setParameters(const OmplReedsSheppPlannerRosParameters& parameters) {
  parameters_ = parameters;
}

bool OmplReedsSheppPlannerRos::initialize() {
  bool result = BASE::initialize();
  initRos();
  initializeStateSpaceBoundaryMarker();
  return result;
}
bool OmplReedsSheppPlannerRos::plan() {
  bool result = BASE::plan();
  if (result) {
    planSeqNumber_++;
  }
  std::thread t([this]() {
    publishPath();
    publishPathNavMsgs();
  });
  t.detach();
  return result;
}

bool OmplReedsSheppPlannerRos::planningService(PlanningService::Request& req, PlanningService::Response& res) {
  // TODO Grid map in state validator is currently initialized to some default values, this check is therefore useless
  if (!planner_->as<OmplReedsSheppPlanner>()->getStateValidator().isInitialized()) {
    ROS_WARN_STREAM("State validator has not been initialized yet. Abort planning.");
    res.status = false;
    return true;
  }

  planTimeStamp_ = req.pathRequest.header.stamp;

  const auto start = se2_planning::convert(req.pathRequest.startingPose);
  const auto goal = se2_planning::convert(req.pathRequest.goalPose);
  setStartingState(start);
  setGoalState(goal);

  // Block update of state validator obstacle map during planning
  planner_->as<OmplReedsSheppPlanner>()->lockStateValidator();
  // TODO move from se2_planner_node here (introduces dependency but makes state space update more consistent:
  //  adapt state space boundaries from grid map
  // planner_->as<OmplReedsSheppPlanner>()->getStateValidator().as<GridMapLazyStateValidator>

  // Adapt state space boundaries (larger than grid map, state validity checking assumes area out of bounds to be
  // traversable) to contain initial and goal state otherwise RRTstar fails
  // TODO expose stateSpaceBoundsMargin_ as param => depends on footprint size?
  // TODO move to OMPLReedsSheppPlanner.cpp?
  const double stateSpaceBoundsMargin_ = parameters_.stateSpaceBoundsMargin_;
  if (!planner_->as<OmplReedsSheppPlanner>()->satisfiesStateSpaceBoundaries(start)) {
    ROS_DEBUG("Initial state not in grid map. Enlarge state space boundaries.");
    const auto bounds = planner_->as<OmplReedsSheppPlanner>()->getStateSpaceBoundaries();
    ompl::base::RealVectorBounds newBounds(reedsSheppStateSpaceDim_);
    newBounds.high[0] = std::max(bounds.high[0], start.x_ + stateSpaceBoundsMargin_);
    newBounds.high[1] = std::max(bounds.high[1], start.y_ + stateSpaceBoundsMargin_);
    newBounds.low[0] = std::min(bounds.low[0], start.x_ - stateSpaceBoundsMargin_);
    newBounds.low[1] = std::min(bounds.low[1], start.y_ - stateSpaceBoundsMargin_);
    planner_->as<OmplReedsSheppPlanner>()->updateStateSpaceBoundaries(newBounds);
  }
  if (!planner_->as<OmplReedsSheppPlanner>()->satisfiesStateSpaceBoundaries(goal)) {
    ROS_DEBUG("Goal state not in grid map. Enlarge state space boundaries.");
    const auto bounds = planner_->as<OmplReedsSheppPlanner>()->getStateSpaceBoundaries();
    ompl::base::RealVectorBounds newBounds(reedsSheppStateSpaceDim_);
    newBounds.high[0] = std::max(bounds.high[0], goal.x_ + stateSpaceBoundsMargin_);
    newBounds.high[1] = std::max(bounds.high[1], goal.y_ + stateSpaceBoundsMargin_);
    newBounds.low[0] = std::min(bounds.low[0], goal.x_ - stateSpaceBoundsMargin_);
    newBounds.low[1] = std::min(bounds.low[1], goal.y_ - stateSpaceBoundsMargin_);
    planner_->as<OmplReedsSheppPlanner>()->updateStateSpaceBoundaries(newBounds);
  }

  // TODO move to detach? Better to publish this info for debugging before checking validity of states.
  publishStartGoalMsgs(start, goal);
  publishStateSpaceBoundaryMarker();

  //  Use state validator only after lock mutex is active and state space is updated
  //  Checks only for non-traversable terrain not for state space bounds
  if (!planner_->as<OmplReedsSheppPlanner>()->getStateValidator().isStateValid(start)) {
    ROS_WARN_STREAM("Start state (x: " << start.x_ << ", y: " << start.y_ << ", yaw: " << start.yaw_
                                       << ") not valid. Start planning anyway.");
  }
  if (!planner_->as<OmplReedsSheppPlanner>()->getStateValidator().isStateValid(goal)) {
    ROS_WARN_STREAM("Goal state (x: " << goal.x_ << ", y: " << goal.y_ << ", yaw: " << goal.yaw_ << ") not valid. Start planning anyway.");
    //    ROS_WARN_STREAM("Goal state (x: " << goal.x_ << ", y: " << goal.y_ << ", yaw: " << goal.yaw_ << ") not valid. Abort planning.");
    //    planner_->as<OmplReedsSheppPlanner>()->unlockStateValidator();
    //    return false;
  }

  bool result = plan();
  planner_->as<OmplReedsSheppPlanner>()->unlockStateValidator();

  res.status = result;

  // Always return true for function, if something fails, set res.status = false, otherwise service call fails with
  // error that service not callable
  return true;
}

void OmplReedsSheppPlannerRos::initRos() {
  pathNavMsgsPublisher_ = nh_->advertise<nav_msgs::Path>(parameters_.pathNavMsgTopic_, 1, true);
  planningService_ = nh_->advertiseService(parameters_.planningSerivceName_, &OmplReedsSheppPlannerRos::planningService, this);
  pathPublisher_ = nh_->advertise<se2_navigation_msgs::PathMsg>(parameters_.pathMsgTopic_, 1);
  startPublisher_ = nh_->advertise<geometry_msgs::PoseStamped>("start", 1);
  goalPublisher_ = nh_->advertise<geometry_msgs::PoseStamped>("goal", 1);
  stateSpacePublisher_ = nh_->advertise<visualization_msgs::Marker>("state_space", 1);
}

void OmplReedsSheppPlannerRos::publishPathNavMsgs() const {
  ReedsSheppPath rsPath;
  planner_->as<OmplPlanner>()->getInterpolatedPath(&rsPath, parameters_.pathNavMsgResolution_);
  nav_msgs::Path msg = se2_planning::copyAllPoints(rsPath);
  msg.header.frame_id = parameters_.pathFrame_;
  msg.header.stamp = planTimeStamp_;
  msg.header.seq = planSeqNumber_;
  pathNavMsgsPublisher_.publish(msg);
  ROS_INFO_STREAM("Publishing ReedsShepp path nav msg, num states: " << msg.poses.size());
}

void OmplReedsSheppPlannerRos::publishPath() const {
  ReedsSheppPath rsPath;
  planner_->getPath(&rsPath);
  se2_navigation_msgs::Path msg = se2_planning::convert(rsPath);
  msg.header_.frame_id = parameters_.pathFrame_;
  msg.header_.stamp = planTimeStamp_;
  msg.header_.seq = planSeqNumber_;
  pathPublisher_.publish(se2_navigation_msgs::convert(msg));
  ROS_INFO_STREAM("Publishing ReedsShepp path, num states: " << rsPath.numPoints());
}

void OmplReedsSheppPlannerRos::publishStartGoalMsgs(const ReedsSheppState& start, const ReedsSheppState& goal) const {
  geometry_msgs::PoseStamped startPose;
  startPose.header.frame_id = parameters_.pathFrame_;
  startPose.header.stamp = planTimeStamp_;
  startPose.pose = se2_planning::convert(start);
  startPublisher_.publish(startPose);
  geometry_msgs::PoseStamped goalPose;
  goalPose.header.frame_id = parameters_.pathFrame_;
  goalPose.header.stamp = planTimeStamp_;
  goalPose.pose = se2_planning::convert(goal);
  goalPublisher_.publish(goalPose);
}

void OmplReedsSheppPlannerRos::initializeStateSpaceBoundaryMarker() {
  double lineWidth = 0.01;
  std_msgs::ColorRGBA color;
  // No transparency
  color.a = 1;
  // Black
  color.r = 0;
  color.g = 0;
  color.b = 0;
  // Init marker
  int nVertices = 5;
  stateSpaceBoundaryMarker_.ns = "state_space";
  stateSpaceBoundaryMarker_.lifetime = ros::Duration();
  stateSpaceBoundaryMarker_.action = visualization_msgs::Marker::ADD;
  stateSpaceBoundaryMarker_.type = visualization_msgs::Marker::LINE_STRIP;
  stateSpaceBoundaryMarker_.scale.x = lineWidth;
  stateSpaceBoundaryMarker_.points.resize(nVertices);  // Initialized to [0.0, 0.0, 0.0]
  stateSpaceBoundaryMarker_.colors.resize(nVertices, color);
}

void OmplReedsSheppPlannerRos::publishStateSpaceBoundaryMarker() {
  // Set marker info.
  stateSpaceBoundaryMarker_.header.frame_id = parameters_.pathFrame_;
  stateSpaceBoundaryMarker_.header.stamp = planTimeStamp_;

  // Set positions of markers.
  const auto bounds = planner_->as<OmplReedsSheppPlanner>()->getStateSpaceBoundaries();
  stateSpaceBoundaryMarker_.points[0].x = bounds.low[0];
  stateSpaceBoundaryMarker_.points[0].y = bounds.low[1];
  stateSpaceBoundaryMarker_.points[1].x = bounds.high[0];
  stateSpaceBoundaryMarker_.points[1].y = bounds.low[1];
  stateSpaceBoundaryMarker_.points[2].x = bounds.high[0];
  stateSpaceBoundaryMarker_.points[2].y = bounds.high[1];
  stateSpaceBoundaryMarker_.points[3].x = bounds.low[0];
  stateSpaceBoundaryMarker_.points[3].y = bounds.high[1];
  // Close the rectangle with the fifth point
  stateSpaceBoundaryMarker_.points[4].x = stateSpaceBoundaryMarker_.points[0].x;
  stateSpaceBoundaryMarker_.points[4].y = stateSpaceBoundaryMarker_.points[0].y;

  stateSpacePublisher_.publish(stateSpaceBoundaryMarker_);
}

} /* namespace se2_planning */

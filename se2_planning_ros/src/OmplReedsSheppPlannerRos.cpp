/*
 * OmplReedsSheppPlannerRos.cpp
 *
 *  Created on: Apr 2, 2020
 *      Author: jelavice
 */

#include "se2_planning_ros/OmplReedsSheppPlannerRos.hpp"

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
  initializeStateSpaceMarker();
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
  // TODO Grid map in state validator is currently initialized default values, this check is therefore useless
  if (!planner_->as<OmplReedsSheppPlanner>()->getStateValidator().isInitialized()) {
    ROS_WARN_STREAM("State validator has not been initialized yet. Abort planning.");
    return false;
  }

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
  if (!planner_->as<OmplReedsSheppPlanner>()->satisfiesStateSpaceBounds(start)) {
    ROS_DEBUG("Initial state not in grid map. Enlarge state space boundaries.");
    const auto bounds = planner_->as<OmplReedsSheppPlanner>()->getStateSpaceBoundaries();
    ompl::base::RealVectorBounds newBounds(reedsSheppStateSpaceDim_);
    newBounds.high[0] = std::max(bounds.high[0], start.x_ + stateSpaceBoundsMargin_);
    newBounds.high[1] = std::max(bounds.high[1], start.y_ + stateSpaceBoundsMargin_);
    newBounds.low[0] = std::min(bounds.low[0], start.x_ - stateSpaceBoundsMargin_);
    newBounds.low[1] = std::min(bounds.low[1], start.y_ - stateSpaceBoundsMargin_);
    planner_->as<OmplReedsSheppPlanner>()->updateStateSpaceBounds(newBounds);
  }
  if (!planner_->as<OmplReedsSheppPlanner>()->satisfiesStateSpaceBounds(goal)) {
    ROS_DEBUG("Goal state not in grid map. Enlarge state space boundaries.");
    const auto bounds = planner_->as<OmplReedsSheppPlanner>()->getStateSpaceBoundaries();
    ompl::base::RealVectorBounds newBounds(reedsSheppStateSpaceDim_);
    newBounds.high[0] = std::max(bounds.high[0], goal.x_ + stateSpaceBoundsMargin_);
    newBounds.high[1] = std::max(bounds.high[1], goal.y_ + stateSpaceBoundsMargin_);
    newBounds.low[0] = std::min(bounds.low[0], goal.x_ - stateSpaceBoundsMargin_);
    newBounds.low[1] = std::min(bounds.low[1], goal.y_ - stateSpaceBoundsMargin_);
    planner_->as<OmplReedsSheppPlanner>()->updateStateSpaceBounds(newBounds);
  }

  // TODO move to detach? Better to publish this info for debugging before checking validity of states.
  publishStartGoalMsgs(start, goal);
  publishStateSpaceMarker();

  //  Use state validator only after lock mutex is active and state space is updated
  //  Checks only for non-traversable terrain not for state space bounds
  if (!planner_->as<OmplReedsSheppPlanner>()->getStateValidator().isStateValid(start)) {
    ROS_WARN_STREAM("Start state (x: " << start.x_ << ", y: " << start.y_ << ", yaw: " << start.yaw_
                                       << ") not valid. Start planning anyway.");
  }
  if (!planner_->as<OmplReedsSheppPlanner>()->getStateValidator().isStateValid(goal)) {
    ROS_WARN_STREAM("Goal state (x: " << goal.x_ << ", y: " << goal.y_ << ", yaw: " << goal.yaw_ << ") not valid. Abort planning.");
    planner_->as<OmplReedsSheppPlanner>()->unlockStateValidator();
    return false;
  }

  bool result = plan();
  planner_->as<OmplReedsSheppPlanner>()->unlockStateValidator();

  res.status = result;

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
  msg.header.stamp = ros::Time::now();
  msg.header.seq = planSeqNumber_;
  pathNavMsgsPublisher_.publish(msg);
  ROS_INFO_STREAM("Publishing ReedsShepp path nav msg, num states: " << msg.poses.size());
}

void OmplReedsSheppPlannerRos::publishPath() const {
  ReedsSheppPath rsPath;
  planner_->getPath(&rsPath);
  se2_navigation_msgs::Path msg = se2_planning::convert(rsPath);
  msg.header_.frame_id = parameters_.pathFrame_;
  msg.header_.stamp = ros::Time::now();
  msg.header_.seq = planSeqNumber_;
  pathPublisher_.publish(se2_navigation_msgs::convert(msg));
  ROS_INFO_STREAM("Publishing ReedsShepp path, num states: " << rsPath.numPoints());
}

void OmplReedsSheppPlannerRos::publishStartGoalMsgs(const ReedsSheppState& start, const ReedsSheppState& goal) const {
  geometry_msgs::PoseStamped startPose;
  startPose.header.frame_id = parameters_.pathFrame_;
  startPose.header.stamp = ros::Time::now();
  startPose.pose = se2_planning::convert(start);
  startPublisher_.publish(startPose);
  geometry_msgs::PoseStamped goalPose;
  goalPose.header.frame_id = parameters_.pathFrame_;
  goalPose.header.stamp = ros::Time::now();
  goalPose.pose = se2_planning::convert(goal);
  goalPublisher_.publish(goalPose);
}

void OmplReedsSheppPlannerRos::initializeStateSpaceMarker() {
  // TODO expose as params?
  double lineWidth_ = 0.1;
  std_msgs::ColorRGBA color_;
  // No transparency
  color_.a = 1;
  // Black
  color_.r = 0;
  color_.g = 0;
  color_.b = 0;
  // Init marker
  int nVertices_ = 5;
  stateSpaceMarker_.ns = "state_space";
  stateSpaceMarker_.lifetime = ros::Duration();
  stateSpaceMarker_.action = visualization_msgs::Marker::ADD;
  stateSpaceMarker_.type = visualization_msgs::Marker::LINE_STRIP;
  stateSpaceMarker_.scale.x = lineWidth_;
  stateSpaceMarker_.points.resize(nVertices_);  // Initialized to [0.0, 0.0, 0.0]
  stateSpaceMarker_.colors.resize(nVertices_, color_);
}

void OmplReedsSheppPlannerRos::publishStateSpaceMarker() {
  // Set marker info.
  stateSpaceMarker_.header.frame_id = parameters_.pathFrame_;
  stateSpaceMarker_.header.stamp = ros::Time::now();

  // Set positions of markers.
  const auto bounds = planner_->as<OmplReedsSheppPlanner>()->getStateSpaceBoundaries();
  stateSpaceMarker_.points[0].x = bounds.low[0];
  stateSpaceMarker_.points[0].y = bounds.low[1];
  stateSpaceMarker_.points[1].x = bounds.high[0];
  stateSpaceMarker_.points[1].y = bounds.low[1];
  stateSpaceMarker_.points[2].x = bounds.high[0];
  stateSpaceMarker_.points[2].y = bounds.high[1];
  stateSpaceMarker_.points[3].x = bounds.low[0];
  stateSpaceMarker_.points[3].y = bounds.high[1];
  // Close the rectangle with the fifth point
  stateSpaceMarker_.points[4].x = stateSpaceMarker_.points[0].x;
  stateSpaceMarker_.points[4].y = stateSpaceMarker_.points[0].y;

  stateSpacePublisher_.publish(stateSpaceMarker_);
}

geometry_msgs::Pose convert(const ReedsSheppState& state, double z) {
  geometry_msgs::Pose pose;
  pose.position.x = state.x_;
  pose.position.y = state.y_;
  pose.position.z = z;
  pose.orientation = tf::createQuaternionMsgFromYaw(state.yaw_);

  return pose;
}

ReedsSheppState convert(const geometry_msgs::Pose& state) {
  ReedsSheppState rsState;
  rsState.x_ = state.position.x;
  rsState.y_ = state.position.y;
  rsState.yaw_ = tf::getYaw(state.orientation);
  return rsState;
}

nav_msgs::Path copyAllPoints(const ReedsSheppPath& path) {
  nav_msgs::Path pathOut;
  pathOut.poses.reserve(path.numPoints());
  for (const auto& segment : path.segment_) {
    for (const auto& point : segment.point_) {
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.pose = convert(point);
      pathOut.poses.push_back(poseStamped);
    }
  }
  return pathOut;
}

se2_navigation_msgs::Path convert(const ReedsSheppPath& path) {
  using DrivingDirection = se2_navigation_msgs::PathSegment::DrivingDirection;
  auto convertDirections = [](ReedsSheppPathSegment::Direction d) -> DrivingDirection {
    switch (d) {
      case ReedsSheppPathSegment::Direction::FWD:
        return DrivingDirection::Forward;
      case ReedsSheppPathSegment::Direction::BCK:
        return DrivingDirection::Backwards;
      default: { throw std::runtime_error("Unknown conversion"); }
    }
  };

  se2_navigation_msgs::Path pathOut;
  pathOut.segment_.reserve(path.segment_.size());
  for (const auto& segment : path.segment_) {
    se2_navigation_msgs::PathSegment segmentOut;
    segmentOut.points_.reserve(segment.point_.size());
    segmentOut.direction_ = convertDirections(segment.direction_);
    for (const auto& point : segment.point_) {
      segmentOut.points_.push_back(convert(point));
    }
    pathOut.segment_.push_back(segmentOut);
  }

  return pathOut;
}

} /* namespace se2_planning */

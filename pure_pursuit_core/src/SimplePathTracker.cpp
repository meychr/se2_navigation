/*
 * SimplePathTracker.cpp
 *
 *  Created on: Mar 24, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_core/path_tracking/SimplePathTracker.hpp"

#include <iostream>

#include "pure_pursuit_core/heading_control/HeadingController.hpp"
#include "pure_pursuit_core/math.hpp"
#include "pure_pursuit_core/path_tracking/PathPreprocessor.hpp"
#include "pure_pursuit_core/path_tracking/ProgressValidator.hpp"
#include "pure_pursuit_core/velocity_control/LongitudinalVelocityController.hpp"

namespace pure_pursuit {

void SimplePathTracker::setParameters(const SimplePathTrackerParameters& parameters) {
  parameters_ = parameters;
}

void SimplePathTracker::importCurrentPath(const Path& path) {
  if (path.segment_.empty()) {
    throw std::runtime_error("empty path");
  }
  currentPath_ = path;
  isPathReceived_ = true;
  currentPathSegmentId_ = 0;
  currentFSMState_ = States::NoOperation;
}

void SimplePathTracker::advanceStateMachine() {
  const bool isSegmentTrackingFinished =
      progressValidator_->isPathSegmentTrackingFinished(currentPath_.segment_.at(currentPathSegmentId_), currentRobotState_);
  const bool isPathTrackingFinished = progressValidator_->isPathTrackingFinished(currentPath_, currentRobotState_, currentPathSegmentId_);

  printFSMState(currentFSMState_);

  if (currentFSMState_ != States::NoOperation && isPathTrackingFinished) {
    currentFSMState_ = States::NoOperation;
    std::cout << "Going to nop state (tracking done)" << std::endl;
  }

  if (currentFSMState_ == States::Driving && isSegmentTrackingFinished) {
    // go to waiting state
    currentFSMState_ = States::Waiting;
    stopwatch_.start();
    const int nSegments = currentPath_.segment_.size();
    currentPathSegmentId_ = bindIndexToRange(currentPathSegmentId_ + 1, 0, nSegments - 1);
    headingController_->updateCurrentPathSegment(currentPath_.segment_.at(currentPathSegmentId_));
    headingController_->initialize();
    velocityController_->updateCurrentPathSegment(currentPath_.segment_.at(currentPathSegmentId_));
    std::cout << "Going to waiting state " << std::endl;
  }

  if (currentFSMState_ == States::Waiting) {
    const bool isWaitedLongEnough = stopwatch_.getElapsedTimeSinceStartSeconds() > parameters_.waitingTimeBetweenDirectionSwitches_;
    if (isWaitedLongEnough) {
      currentFSMState_ = States::Driving;
      std::cout << "Going to driving state (done waiting)" << std::endl;
    }
  }

  if (currentFSMState_ == States::NoOperation && isPathReceived_) {
    currentFSMState_ = States::Driving;
    headingController_->updateCurrentPathSegment(currentPath_.segment_.at(currentPathSegmentId_));
    headingController_->initialize();
    velocityController_->updateCurrentPathSegment(currentPath_.segment_.at(currentPathSegmentId_));
    std::cout << "Going to driving state (received plan)" << std::endl;
  }

  if (!currentPath_.segment_.empty()) {
    currentDrivingDirection_ = currentPath_.segment_.at(currentPathSegmentId_).drivingDirection_;
  }

  isPathReceived_ = false;
}

bool SimplePathTracker::advanceControllers() {
  bool result = true;
  velocityController_->updateCurrentState(currentRobotState_);
  velocityController_->updateDrivingDirection(currentDrivingDirection_);
  headingController_->updateCurrentState(currentRobotState_);
  headingController_->updateDesiredVelocity(Vector(longitudinalVelocity_, 0.0));
  switch (currentFSMState_) {
    case States::Driving: {
      const bool velControllerStatus = velocityController_->advance();
      result = result && velControllerStatus;
      const bool headingControllerStatus = headingController_->advance();
      result = result && headingControllerStatus;
      longitudinalVelocity_ = velocityController_->getVelocity();
      break;
    }
    case States::NoOperation: {
      longitudinalVelocity_ = 0.0;
      break;
    }
    case States::Waiting: {
      longitudinalVelocity_ = 0.0;
      result = result && headingController_->advance();
      break;
    }
  }

  turningRadius_ = headingController_->getTurningRadius();
  yawRate_ = headingController_->getYawRate();
  steeringAngle_ = headingController_->getSteeringAngle();
  //

  result = result && std::isfinite(turningRadius_);
  result = result && std::isfinite(yawRate_);
  result = result && std::isfinite(steeringAngle_);

  return result;
}

void SimplePathTracker::stopTracking() {
  currentFSMState_ = States::NoOperation;
}

void SimplePathTracker::printFSMState(States FSMState) {
  // Only print state if it changed from the last function call
  static States previousFSMState = States::NoOperation;

  if (previousFSMState != FSMState) {
    switch (FSMState) {
      case States::NoOperation:
        std::cout << "States::NoOperation" << std::endl;
        break;
      case States::Driving:
        std::cout << "States::Driving" << std::endl;
        break;
      case States::Waiting:
        std::cout << "States::Waiting" << std::endl;
        break;
      default:
        std::cout << "No valid state" << std::endl;
    }
    previousFSMState = FSMState;
  }
}

std::unique_ptr<PathTracker> createSimplePathTracker(const SimplePathTrackerParameters& parameters,
                                                     std::shared_ptr<LongitudinalVelocityController> velocityController,
                                                     std::shared_ptr<HeadingController> headingController,
                                                     std::shared_ptr<ProgressValidator> validator,
                                                     std::shared_ptr<PathPreprocessor> pathPreprocessor) {
  std::unique_ptr<SimplePathTracker> tracker = std::make_unique<SimplePathTracker>();
  tracker->setParameters(parameters);
  tracker->setHeadingController(headingController);
  tracker->setVelocityController(velocityController);
  tracker->setProgressValidator(validator);
  tracker->setPathPreprocessor(pathPreprocessor);
  return std::move(tracker);
}

} /* namespace pure_pursuit */

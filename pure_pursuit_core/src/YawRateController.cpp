/*
 * YawRateController.cpp
 *
 *  Created on: Jul 06, 2020
 *      Author: meyerc, jelavice
 */

#include "pure_pursuit_core/heading_control/YawRateController.hpp"

#include <iostream>

#include "pure_pursuit_core/math.hpp"

namespace pure_pursuit {

bool YawRateController::advanceImpl() {
  chooseActiveAnchorAndLookaheadDistance(parameters_);
  const auto drivingDirection = currentPathSegment_.drivingDirection_;
  const auto& robotPose = currentRobotState_.pose_;
  const Point anchorPoint = computeAnchorPoint(currentRobotState_.pose_, activeAnchorDistance_, drivingDirection);
  currentAnchorPoint_ = anchorPoint;
  const unsigned int closestPointOnPathId = getIdOfTheClosestPointOnThePath(currentPathSegment_, robotPose.position_, lastClosestPointId_);

  Point lookaheadPoint;
  if (!computeLookaheadPoint(closestPointOnPathId, activeLookaheadDistance_, currentRobotState_, drivingDirection, currentPathSegment_,
                             activeAnchorDistance_, &lookaheadPoint)) {
    std::cerr << "YawRateController: Failed to compute lookahead point." << std::endl;
    return false;
  }
  currentLookaheadPoint_ = lookaheadPoint;

  const auto heading = computeDesiredHeadingVector(robotPose.yaw_, drivingDirection);
  // Always use DrivingDirection::FWD here because for Spacebok/pure yaw rate control the driving direction has no
  // influence, for Ackermann steering the convention is different for forward and backward drive
  if (!computeLookaheadAngle(lookaheadPoint, anchorPoint, heading, DrivingDirection::FWD, &lookaheadAngle_)) {
    std::cerr << "YawRateController: Failed to compute lookahead angle" << std::endl;
    return false;
  }

  const double steeringAngle =
      computeSteeringAngleCmd(lookaheadAngle_, activeLookaheadDistance_, activeAnchorDistance_, parameters_.wheelBase_);

  if (std::isnan(steeringAngle) || std::isinf(steeringAngle)) {
    std::cerr << "YawRateController: Computed steering angle is nan" << std::endl;
    return false;
  }

  const double filtered = avgFilter_.filterInputValue(steeringAngle);
  const double deadZoned = deadZone(filtered, parameters_.deadZoneWidth_);
  const double rateLimited = rateLimiter_.limitRateOfChange(deadZoned);
  const double boundToRange = bindToRange(rateLimited, -parameters_.maxSteeringAngleMagnitude_, parameters_.maxSteeringAngleMagnitude_);
  steeringAngle_ = boundToRange;
  lastClosestPointId_ = closestPointOnPathId;
  return true;
}

bool YawRateController::initialize() {
  lastClosestPointId_ = 0.0;
  return true;
}

bool YawRateController::computeSteeringAngle() {
  return true;
}
bool YawRateController::computeYawRate() {
  const double v = desiredLinearVelocity_.norm();
  yawRate_ = 2.0 * v * std::sin(lookaheadAngle_) / activeLookaheadDistance_;
  return true;
}
bool YawRateController::computeTurningRadius() {
  const double v = desiredLinearVelocity_.norm();
  const double yawRate = v / parameters_.wheelBase_ * std::tan(steeringAngle_);
  turningRadius_ = v / (std::fabs(yawRate) + 1e-4) * sgn(yawRate);
  return true;
}

void YawRateController::updateCurrentPathSegment(const PathSegment& pathSegment) {
  lastClosestPointId_ = 0;  // reset
  currentPathSegment_ = pathSegment;
  const double extendingLength = 2.0 * std::max(parameters_.lookaheadDistanceFwd_, parameters_.anchorDistanceBck_);
  appendPointAlongFinalApproachDirection(extendingLength, &currentPathSegment_);
}

void YawRateController::setParameters(const YawRateCtrlParameters& parameters) {
  if (parameters.anchorDistanceBck_ < 0) {
    throw std::runtime_error("anchorDistanceBck_ is less than 0.");
  }

  if (parameters.anchorDistanceFwd_ < 0) {
    throw std::runtime_error("anchorDistanceFwd_ is less than 0.");
  }

  if (parameters.lookaheadDistanceBck_ < 0) {
    throw std::runtime_error("lookaheadDistanceBck_ is less than 0.");
  }

  if (parameters.lookaheadDistanceFwd_ < 0) {
    throw std::runtime_error("lookaheadDistanceFwd_ is less than 0.");
  }

  if (parameters.wheelBase_ < 0) {
    throw std::runtime_error("wheelBase_ is less than 0.");
  }

  if (parameters.maxSteeringAngleMagnitude_ < 0) {
    throw std::runtime_error("maxSteeringAngleMagnitude_ is less than 0.");
  }

  if (parameters.maxSteeringRateOfChange_ < 0) {
    throw std::runtime_error("maxSteeringRateOfChange_ is less than 0.");
  }

  if (parameters.deadZoneWidth_ < 0) {
    throw std::runtime_error("deadZoneWidth_ is less than 0.");
  }

  parameters_ = parameters;
  rateLimiter_.setTimestep(parameters.dt_);
  rateLimiter_.setFallingRate(-parameters.maxSteeringRateOfChange_);
  rateLimiter_.setRisingRate(parameters.maxSteeringRateOfChange_);

  avgFilter_.setWeightForMostRecentMeasurement(parameters.avgFilgerCurrentSampleWeight_);
}

YawRateCtrlParameters YawRateController::getParameters() const {
  return parameters_;
}

std::unique_ptr<HeadingController> createYawRateController(const YawRateCtrlParameters& parameters) {
  std::unique_ptr<YawRateController> controller = std::make_unique<YawRateController>();
  controller->setParameters(parameters);
  return std::move(controller);
}

} /* namespace pure_pursuit */

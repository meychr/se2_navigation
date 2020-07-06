/*
 * YawRateController.hpp
 *
 *  Created on: Jul 06, 2020
 *      Author: meyerc, jelavice
 */

#pragma once

#include <memory>

#include "pure_pursuit_core/heading_control/HeadingController.hpp"
#include "pure_pursuit_core/math.hpp"

namespace pure_pursuit {

struct YawRateCtrlParameters : public HeadingControllerParameters {
  double wheelBase_ = 4.0;
  double maxSteeringAngleMagnitude_ = 0.5;  // rad
  double maxSteeringRateOfChange_ = 0.2;    // rad/s
  double dt_ = 0.01;                        // seconds
};

class YawRateController : public HeadingController {
 public:
  YawRateController() = default;
  ~YawRateController() override = default;

  void setParameters(const YawRateCtrlParameters& parameters);
  YawRateCtrlParameters getParameters() const;
  void updateCurrentPathSegment(const PathSegment& pathSegment) override;
  bool initialize() override;

 protected:
  bool advanceImpl() override;
  bool computeYawRate() override;
  bool computeTurningRadius() override;
  bool computeSteeringAngle() override;

  YawRateCtrlParameters parameters_;
  RateLimiter rateLimiter_;
  AverageFilter avgFilter_;
  Point currentAnchorPoint_, currentLookaheadPoint_;

  double cmd_ = 0.0;
  double lookaheadAngle_ = 0.0;
};

std::unique_ptr<HeadingController> createYawRateController(const YawRateCtrlParameters& parameters);

}  // namespace pure_pursuit

/*
 * YawRateControllerRos.hpp
 *
 *  Created on: Jul 06, 2020
 *      Author: meyerc, jelavice
 */

#pragma once

#include <ros/ros.h>
#include "pure_pursuit_core/heading_control/YawRateController.hpp"

namespace pure_pursuit {

class YawRateControllerRos : public YawRateController {
  using BASE = YawRateController;

 public:
  explicit YawRateControllerRos(ros::NodeHandle* nh);

 private:
  void initRos();
  bool advanceImpl() override;
  void publishAnchorPoint() const;
  void publishPathSegment() const;
  void publishLookaheadPoint() const;
  void publishP1() const;
  void publishP2() const;

  ros::NodeHandle* nh_;
  ros::Publisher lookaheadPointPub_;
  ros::Publisher anchorPointPub_;
  ros::Publisher pathSegmentPub_;

  ros::Publisher p1Pub_;
  ros::Publisher p2Pub_;
  Point p1_, p2_;
};

std::unique_ptr<HeadingController> createYawRateControllerRos(const YawRateCtrlParameters& parameters, ros::NodeHandle* nh);

} /* namespace pure_pursuit */

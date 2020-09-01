//
// Created by christoph on 16.07.20.
//

#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace se2_planning {

class GridMapTest {
public:
  GridMapTest(ros::NodeHandlePtr nh);
  ~GridMapTest() = default;

  void initRos();
  bool initialize();
  void publishMap();
  bool loadParameters();

protected:
  void obstacleCb(geometry_msgs::Point position);
  void positionCb(geometry_msgs::Point position);

  ros::NodeHandlePtr nh_;
  ros::Subscriber obstacleSub_;
  ros::Subscriber positionSub_;
  ros::Publisher mapPub_;
  grid_map::GridMap map_;

private:
  std::string mapFrameId_;
  std::string layerName_;
  double mapResolution_;
  double mapPositionX_;
  double mapPositionY_;
  double mapLength_;
  double mapWidth_;
  double obstacleLength_;
  double obstacleWidth_;
};

} /* namespace se2_planning */
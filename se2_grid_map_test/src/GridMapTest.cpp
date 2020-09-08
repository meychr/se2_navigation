//
// Created by christoph on 16.07.20.
//

#include "se2_grid_map_test/GridMapTest.hpp"

namespace se2_planning {

GridMapTest::GridMapTest(ros::NodeHandlePtr nh) : nh_(nh) {}

void GridMapTest::initRos(){
  mapPub_ = nh_->advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  obstacleSub_ = nh_->subscribe("obstacle", 1, &GridMapTest::obstacleCb, this);
  positionSub_ = nh_->subscribe("position", 1, &GridMapTest::positionCb, this);
}

bool GridMapTest::loadParameters(){
  if (!nh_->getParam("map/frame_id", mapFrameId_)) return false;
  if (!nh_->getParam("map/layer_name", layerName_)) return false;
  if (!nh_->getParam("map/resolution", mapResolution_)) return false;
  if (!nh_->getParam("map/position/x", mapPositionX_)) return false;
  if (!nh_->getParam("map/position/y", mapPositionY_)) return false;
  if (!nh_->getParam("map/length", mapLength_)) return false;
  if (!nh_->getParam("map/width", mapWidth_)) return false;
  if (!nh_->getParam("obstacle/length", obstacleLength_)) return false;
  if (!nh_->getParam("obstacle/width", obstacleWidth_)) return false;
  if (obstacleLength_ / 2.0 < mapResolution_) {
    ROS_ERROR_STREAM("obstacle_length " << obstacleLength_ << " is too small for chosen map_resolution "
                      << mapResolution_ << ". Minimum is two times the map_resolution.");
    return false;
  }
  if (obstacleWidth_ / 2.0 < mapResolution_) {
    ROS_ERROR_STREAM("obstacle_width " << obstacleWidth_ << " is too small for chosen map_resolution "
                      << mapResolution_ << ". Minimum is two times the map_resolution.");
    return false;
  }
  return true;
}

bool GridMapTest::initialize() {
  if (!loadParameters()) {
    ROS_ERROR("ROS parameters could not be loaded.");
  }
  initRos();
  initMap();
  publishMap();
}

void GridMapTest::initMap() {
  map_.setFrameId(mapFrameId_);
  map_.setTimestamp(ros::Time::now().toNSec());
  map_.setGeometry(grid_map::Length(mapLength_, mapWidth_), mapResolution_,
                   grid_map::Position(mapPositionX_, mapPositionY_));
  map_.add(layerName_, 0.0);
}

void GridMapTest::publishMap() {
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(map_, msg);
  mapPub_.publish(msg);
}

void GridMapTest::obstacleCb(geometry_msgs::Point position) {
  double x = position.x;
  double y = position.y;

  map_[layerName_].setConstant(0.0);

  // Add obstacles to traversability layer
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    map_.getPosition(*iterator, position);
    if (position.x() < (x + obstacleLength_ / 2.0) && position.x() > (x - obstacleLength_ / 2.0)
        && position.y() < (y + obstacleWidth_ / 2.0) && position.y() > (y - obstacleWidth_ / 2.0)) {
      map_.at(layerName_, *iterator) = 1.0;  // obstacles
    }
  }

  publishMap();
}

void GridMapTest::positionCb(geometry_msgs::Point position) {
  grid_map::Position mapPosition;
  mapPosition.x() = position.x;
  mapPosition.y() = position.y;
  map_.setPosition(mapPosition);
  publishMap();
}

} /* namespace se2_planning */
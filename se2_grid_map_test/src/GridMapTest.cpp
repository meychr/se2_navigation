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

void GridMapTest::loadParameters(){
  mapFrameId_ = "world";
  layerName_ = "traversability";
  mapResolution_ = 0.1;
  obstacleLength_ = 3;
  obstacleWidth_ = 4;
}

bool GridMapTest::initialize() {
  loadParameters();
  initRos();

  double mapLengthX = 20.0;
  double mapLengthY = 20.0;
  map_.setFrameId(mapFrameId_);
  map_.setTimestamp(ros::Time::now().toNSec());
  map_.setGeometry(grid_map::Length(mapLengthX, mapLengthY), mapResolution_,
                   grid_map::Position(0, 0));  // adjust planner parameters as well!
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
    if (position.x() < (x+obstacleLength_) && position.x() > x && fabs(position.y()) < obstacleWidth_) {
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
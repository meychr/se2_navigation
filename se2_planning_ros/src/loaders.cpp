/*
 * loaders.cpp
 *
 *  Created on: Apr 3, 2020
 *      Author: jelavice
 */

#include "se2_planning_ros/loaders.hpp"

#include <yaml-cpp/yaml.h>
#include <stdexcept>

namespace se2_planning {

OmplReedsSheppPlannerParameters loadOmplReedsSheppPlannerParameters(const std::string& filename) {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("OmplReedsSheppPlannerParameters::loadParameters loading failed");
  }

  OmplReedsSheppPlannerParameters parameters;
  {
    auto node = basenode["state_space"];
    parameters.xLowerBound_ = node["x_lower"].as<double>();
    parameters.xUpperBound_ = node["x_upper"].as<double>();
    parameters.yLowerBound_ = node["y_lower"].as<double>();
    parameters.yUpperBound_ = node["y_upper"].as<double>();
    parameters.turningRadius_ = node["turning_radius"].as<double>();
  }

  {
    auto node = basenode["planner"];
    parameters.pathSpatialResolution_ = node["path_spatial_resolution"].as<double>();
    parameters.maxPlanningTime_ = node["max_planning_time"].as<double>();
  }

  return parameters;
}

OmplReedsSheppPlannerRosParameters loadOmplReedsSheppPlannerRosParameters(const std::string& filename) {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("OmplReedsSheppPlannerRosParameters::loadParameters loading failed");
  }

  OmplReedsSheppPlannerRosParameters parameters;
  auto node = basenode["planner_ros"];
  parameters.pathNavMsgTopic_ = node["nav_msgs_path_topic"].as<std::string>();
  parameters.pathFrame_ = node["path_frame"].as<std::string>();
  parameters.planningSerivceName_ = node["planning_service_name"].as<std::string>();
  parameters.pathNavMsgResolution_ = node["nav_msg_path_spatial_resolution"].as<double>();
  parameters.pathMsgTopic_ = node["path_msg_topic"].as<std::string>();

  return parameters;
}

RRTstarParameters loadRRTstarParameters(const std::string& filename) {
  RRTstarParameters parameters;

  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("loadRRTstarParameters failed");
  }

  auto node = basenode["ompl_planners"]["rrt_star"];
  parameters.range_ = node["planner_range"].as<double>();

  return parameters;
}
RRTsharpParameters loadRRTsharpParameters(const std::string& filename) {
  RRTsharpParameters parameters;

  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("loadRRTstarParameters failed");
  }

  auto node = basenode["ompl_planners"]["rrt_sharp"];
  parameters.range_ = node["planner_range"].as<double>();

  return parameters;
}

} /* namespace se2_planning */

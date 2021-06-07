/*
 * HeightMap.hpp
 *
 *  Created on: Feb 17, 2021
 *      Author: jelavice
 */

#pragma once
#include "grid_map_core/GridMap.hpp"
#include "se2_planning/Map.hpp"

namespace se2_planning {

class HeightMap : public Map {
 public:
  HeightMap() = default;
  ~HeightMap() override = default;

  bool isInitialized() const override;
  void initializeBounds(const Bounds& bounds) override;
  Bounds getBounds() const override;
  double getValueAt(double x, double y) const override;
  void setGridMap(const grid_map::GridMap& gm, const std::string& heightLayer);

 private:
  grid_map::GridMap impl_;
  std::string heightLayer_ = "";
  bool isInitialized_ = false;
};

} /* namespace se2_planning*/

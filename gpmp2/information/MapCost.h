/**
 *  @file  MapCost.h
 *  @brief map cost functions, implement map information cost function
 *  @author Taekyun Kim
 *  @date  May 30, 2020
 **/

#pragma once

#include <gpmp2/information/GridMap.h>
#include <gpmp2/information/MapException.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <iostream>


namespace gpmp2 {


/// hinge loss obstacle cost function
inline double MapCost(const gtsam::Point2& point, const GridMap& grid_map,
  gtsam::OptionalJacobian<1, 2> H_point = boost::none) {

  gtsam::Vector2 map_gradient;
  double information;
  try {
    information = grid_map.getUpdatedInformation(point, map_gradient);
  } catch (MapQueryOutOfRange&) {

    std::cout << "[MapCost] WARNING: querying grid map out of range, "
        "assume zero information cost." << std::endl;
    if (H_point) *H_point = gtsam::Matrix12::Zero();
    return 0.0;
  }
  if (H_point) *H_point = -map_gradient.transpose();
  return information;
}
}



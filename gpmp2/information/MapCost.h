/**
 *  @file  MapCost.h
 *  @brief map cost functions, implement map information cost function
 *  @author Taekyun Kim
 *  @date  May 30, 2020
 **/

#pragma once

#include <gpmp2/information/GridMap.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <iostream>


namespace gpmp2 {


/// hinge loss obstacle cost function
inline double MapCost(const gtsam::Point2& point, const GridMap& grid_map,
    double coefficient, gtsam::OptionalJacobian<1, 3> H_point = boost::none) {
  double information = grid_map.getUpdatedInformation(point);
  return information * coefficient;
}
}



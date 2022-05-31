/**
 *  @file  MapException.h
 *  @brief custom exceptions for grid map
 *  @author Taekyun Kim
 *  @date  May 30, 2020
 **/

#pragma once

#include <gpmp2/config.h>
#include <stdexcept>


namespace gpmp2 {

/// query out of range exception
class GPMP2_EXPORT MapQueryOutOfRange : public std::runtime_error {

public:
  /// constructor
  MapQueryOutOfRange() : std::runtime_error("Querying Map out of range") {}
};

}



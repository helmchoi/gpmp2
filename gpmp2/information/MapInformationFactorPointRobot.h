/**
 *  @file   MapInformationFactorPointRobot.h
 *  @brief  Map information cost factor, for point robot, using grid map
 *  @author Taekyun Kim
 *  @date   May 30, 2020
 **/

#pragma once

#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/information/MapInformationFactor.h>

namespace gpmp2 {

// template uses PointRobotModel as robot type
typedef MapInformationFactor<PointRobotModel> MapInformationFactorPointRobot;

}

/**
 *  @file  MapInformationFactor.h
 *  @brief Map information factor
 *  @author Taekyun Kim
 *  @date  May 30, 2020
 **/


#pragma once

#include <gpmp2/information/GridMap.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <iostream>
#include <vector>


namespace gpmp2{

/**
 * unary factor for obstacle avoidance, planar version
 * template robot model version
 */
template <class ROBOT>
class MapInformationFactor: public gtsam::NoiseModelFactor1<typename ROBOT::Pose> {

public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;

private:
  // typedefs
  typedef MapInformationFactor This;
  typedef gtsam::NoiseModelFactor1<Pose> Base;

  // arm: planar one, all alpha = 0
  const Robot& robot_;

  // signed distance field from matlab
  const GridMap& grid_map_;


public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /* Default constructor do nothing */
  MapInformationFactor() : robot_(Robot()), grid_map_(GridMap()) {}

  /**
   * Constructor
   * @param cost_model cost function covariance, should to identity model
   * @param map      signed distance field
   * @param nn_index   nearest neighbour index of grid map
   */
  MapInformationFactor(gtsam::Key poseKey, const Robot& robot,
      const GridMap& grid_map, double cost_sigma) :
        Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(), cost_sigma), poseKey),
        robot_(robot), grid_map_(grid_map) {

     // TODO: check robot is plannar
  }

  virtual ~MapInformationFactor() {}


  /// error function
  /// numerical jacobians / analytic jacobians from cost function
  gtsam::Vector evaluateError(const Pose& conf,
      boost::optional<gtsam::Matrix&> H1 = boost::none) const ;


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "MapInformationFactor :" << std::endl;
    Base::print("", keyFormatter);
  }


  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor1",
        boost::serialization::base_object<Base>(*this));
  }
};

}

#include <gpmp2/information/MapInformationFactor-inl.h>

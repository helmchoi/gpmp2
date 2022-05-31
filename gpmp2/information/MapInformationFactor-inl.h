/**
 *  @file  MapInformationFactor-inl.h
 *  @brief Map information factor
 *  @author Taekyun Kim
 *  @date  May 30, 2020
 **/

#include <gpmp2/information/MapCost.h>

using namespace std;
using namespace gtsam;


namespace gpmp2 {

/* ************************************************************************** */
template <class ROBOT>
gtsam::Vector MapInformationFactor<ROBOT>::evaluateError(
    const Pose& conf, boost::optional<gtsam::Matrix&> H1) const {

  if (H1) *H1 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());

  vector<Point3> sph_centers;
  vector<Matrix> J_px_jp;
  if (H1)
    robot_.sphereCenters(conf, sph_centers, J_px_jp);
  else
    robot_.sphereCenters(conf, sph_centers);

  // allocate cost vector
  Vector err(robot_.nr_body_spheres());

  // for each point on arm stick, get error
  for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {
    const Point2 sph_center_2d(sph_centers[sph_idx].x(), sph_centers[sph_idx].y());

    if (H1) {
      Matrix12 Jerr_point;

      err(sph_idx) = MapCost(sph_center_2d, grid_map_, Jerr_point);
      H1->row(sph_idx) = Jerr_point * J_px_jp[sph_idx].topRows<2>();
    } else {
      err(sph_idx) = MapCost(sph_center_2d, grid_map_);
    }
  }

  return err;
}

}

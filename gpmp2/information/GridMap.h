/**
 *  @file  GridMap.h
 *  @brief util functions for grid map
 *  @author Taekyun Kim
 *  @date  May 30, 2020
 **/

#pragma once

#include <gpmp2/information/MapException.h>
#include <gpmp2/config.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>

#include <iostream>


namespace gpmp2 {

/**
 * grid map use Matrix as data type
 * Matrix represent the X (col) & Y (row) dimension
 */
class GPMP2_EXPORT GridMap {

public:
  // index and float_index is <row, col>
  typedef boost::tuple<size_t, size_t> index;
  typedef boost::tuple<double, double> float_index;
  typedef boost::shared_ptr<GridMap> shared_ptr;

private:
  gtsam::Point2 origin_;
  // geometry setting of grid map
  size_t map_rows_, map_cols_;
  double cell_size_;
  gtsam::Matrix data_;

public:
  /// constructor
  GridMap() : map_rows_(0), map_cols_(0), cell_size_(0.0) {}

  /// constructor with data
  GridMap(const gtsam::Point2& origin, double cell_size, const gtsam::Matrix& data):
    origin_(origin), map_rows_(data.rows()), map_cols_(data.cols()),
    cell_size_(cell_size), data_(data) {}
  
  ~GridMap() {}

  /// give a point, search for signed distance field and (optional) gradient
  /// return signed distance
  inline double getUpdatedInformation(const gtsam::Point2& point) const {
    const float_index pidx = convertPoint2toCell(point);
    return updated_information(pidx);
  }

  inline double getUpdatedInformation(const gtsam::Point2& point, gtsam::Vector2& g) const {
    const float_index pidx = convertPoint2toCell(point);
    const gtsam::Vector2 g_idx = gradient(pidx);
    g = gtsam::Vector2(g_idx(1), g_idx(0)) / cell_size_;
    return updated_information(pidx);
  }

  inline float_index convertPoint2toCell(const gtsam::Point2& point) const {
    // check point range
    if (point.x() < origin_.x() || point.x() > (origin_.x() + (map_cols_-1.0)*cell_size_) ||
        point.y() < origin_.y() || point.y() > (origin_.y() + (map_rows_-1.0)*cell_size_)) {
        
      throw MapQueryOutOfRange();
    }

    const double col = (point.x() - origin_.x()) / cell_size_;
    const double row = (point.y() - origin_.y()) / cell_size_;
    return boost::make_tuple(row, col);
  }

  inline gtsam::Point2 convertCelltoPoint2(const float_index& cell) const {
    return origin_ + gtsam::Point2(
        cell.get<1>() * cell_size_,
        cell.get<0>() * cell_size_);
  }

  inline gtsam::Vector2 gradient(const float_index& idx) const {
    const float lr = floor(idx.get<0>()) - 1.0, lc = floor(idx.get<1>()) - 1.0;
    const float hr = lr + 1.0, hc = lc + 1.0;
    const float_index hrlc(hr, lc);
    const float_index lrlc(lr, lc);
    const float_index lrhc(lr, hc);
    const float_index hrhc(hr, hc);
    const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc),
        hri = static_cast<size_t>(hr), hci = static_cast<size_t>(hc);
    return gtsam::Vector2(
        (hc-idx.get<1>()) * (updated_information(hrlc)-updated_information(lrlc)) +
        (idx.get<1>()-lc) * (updated_information(hrhc)-updated_information(lrhc)),

        (hr-idx.get<0>()) * (updated_information(lrhc)-updated_information(lrlc)) +
        (idx.get<0>()-lr) * (updated_information(hrhc)-updated_information(hrlc)));
  }


  // ======================================================================
  // Computation of information cost
  // information gain w.r.t. the current map (each timestep is independent)
  // ======================================================================
  //
  // -- Calculate information cost (TK)
//   inline double updated_information(const float_index& idx) const {
//     const size_t n = 100;
//     const double range = 10.0;
//     const double dtheta = 2.0 * M_PI / float(n);
//     const double stride = cell_size_ * 0.9;
//     const size_t n_d = size_t(range / stride);
//     double cost = 0.0;
//     for (size_t i = 0; i < n ; i++){
//       const double theta_k = dtheta * i;
//       double y = idx.get<0>();
//       double x = idx.get<1>();
//       for (size_t j = 0; j < n_d; j++){
//         x -= stride * std::sin(theta_k);
//         y += stride * std::cos(theta_k);
//         const size_t cell_idx_x = static_cast<size_t>(x);
//         const size_t cell_idx_y = static_cast<size_t>(y);
//         if(cell_idx_x < 0 || cell_idx_y < 0 || cell_idx_x > map_cols_ || cell_idx_y > map_rows_){
//           cost += n_d - j;
//           break;
//         }
//         if(data_(cell_idx_x, cell_idx_y) == 1){
//           cost += n_d - j;
//           break;
//         } else if(data_(cell_idx_x, cell_idx_y) == 0){
//           cost += 1;
//         }
//       }
//     }
//     return cost;
//   }
  
  // -- Calculate information cost (HL)
  inline double updated_information(const float_index& idx) const {
    const size_t n = 100;
    const double range = 10.0;
    const double dtheta = 2.0 * M_PI / float(n);
    const double stride = cell_size_ * 0.9;
    const size_t n_d = size_t(range / stride);
    // double cost = 0.0;
    double cost = -0.5 * log(0.5) * M_PI * range * range;
    for (size_t i = 0; i < n ; i++){
      const double theta_k = dtheta * i;
      double y = idx.get<0>();
      double x = idx.get<1>();
      for (size_t j = 0; j < n_d; j++){
        x -= stride * std::sin(theta_k);
        y += stride * std::cos(theta_k);
        const size_t cell_idx_x = static_cast<size_t>(x);
        const size_t cell_idx_y = static_cast<size_t>(y);
        // out of the map range
        if(cell_idx_x < 0 || cell_idx_y < 0 || cell_idx_x > map_cols_ || cell_idx_y > map_rows_){
          break;
        }
        // this cell is known to be occupied (then the farther cells are not seen) -> already known. no gain
        if(data_(cell_idx_x, cell_idx_y) == 1){
          break;
        }
        // // this cell is known to be free -> already known. no gain
        // else if(data_(cell_idx_x, cell_idx_y) == 0){
        // }
        // this cell is unknown (now it can be observed -> give info gain)
        else if(data_(cell_idx_x, cell_idx_y) == 0.5){
          cost += 0.5 * log(0.5);
        }
      }
    }
    return cost;
  }

  const gtsam::Point2& origin() const { return origin_; }
  size_t x_count() const { return map_cols_; }
  size_t y_count() const { return map_rows_; }
  double cell_size() const { return cell_size_; }
  const gtsam::Matrix& raw_data() const { return data_; }

  /// print
  void print(const std::string& str = "") const {
    std::cout << str;
    std::cout << "map origin:     ";
    origin_.print();
    std::cout << "map resolution: " << cell_size_ << std::endl;
    std::cout << "map size:       " << map_cols_ << " x "
        << map_rows_ << std::endl;
  }

};

}

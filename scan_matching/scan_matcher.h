#ifndef SCAN_MATCHER_H_
#define SCAN_MATCHER_H_

#include <iostream>
#include <vector>
#include "eigen3/Eigen/Dense"

class ScanMatcher {
 public:
  ~ScanMatcher(){};

  virtual bool OptimizeScanPose(
      const uint8_t* probability_map_ptr, const int map_rows,
      const int map_cols, const double map_resolution,
      const std::vector<Eigen::Vector2d>& scan_point_list,
      Eigen::Vector3d* scan_pose_ptr) = 0;

 private:
};

#endif  // SCAN_MATCHER_ANALYTIC_H_
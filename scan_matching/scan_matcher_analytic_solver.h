#ifndef SCAN_MATCHER_ANALYTIC_SOLVER_H_
#define SCAN_MATCHER_ANALYTIC_SOLVER_H_

#include "eigen3/Eigen/Dense"
#include "scan_matcher.h"

class ScanMatcherAnalyticSolver : public ScanMatcher {
 public:
  ScanMatcherAnalyticSolver();
  ~ScanMatcherAnalyticSolver();

  bool OptimizeScanPose(const uint8_t* probability_map_ptr, const int map_rows,
                        const int map_cols, const double map_resolution,
                        const std::vector<Eigen::Vector2d>& scan_point_list,
                        Eigen::Vector3d* scan_pose_ptr) final;
};

#endif  // SCAN_MATCHER_ANALYTIC_H_
#include "scan_matcher_analytic_solver.h"

ScanMatcherAnalyticSolver::ScanMatcherAnalyticSolver() {}

ScanMatcherAnalyticSolver::~ScanMatcherAnalyticSolver() {}

bool ScanMatcherAnalyticSolver::OptimizeScanPose(
    const uint8_t* probability_map_ptr, const int probability_map_height,
    const int probability_map_width, const double map_resolution,
    const std::vector<Eigen::Vector2d>& scan_point_list,
    Eigen::Vector3d* scan_pose_ptr) {
  bool is_success = true;

  // Reserve initial pose for recovery
  const auto optimization_pose_parameters = *scan_pose_ptr;

  // Generate warped point buffers
  const int num_points = static_cast<int>(scan_point_list.size());
  std::vector<double> warped_x_list(num_points, 0);
  std::vector<double> warped_y_list(num_points, 0);

  // Do optimization iterations
  static constexpr int kMaxIteration{100};
  for (int iteration = 0; iteration < kMaxIteration; ++iteration) {
    // Get current parameters
    const auto x_curr = optimization_pose_parameters(0);
    const auto y_curr = optimization_pose_parameters(1);
    const auto yaw_curr = optimization_pose_parameters(2);

    const double cos_yaw = std::cos(yaw_curr);
    const double sin_yaw = std::sin(yaw_curr);

    // Warp points
    for (int index = 0; index < num_points; ++index) {
      const auto& point = scan_point_list[index];
      warped_x_list[index] = cos_yaw * point.x() - sin_yaw * point.y() + x_curr;
      warped_y_list[index] = sin_yaw * point.x() + cos_yaw * point.y() + y_curr;
    }

    // Calculate probabilities for warped points

    // Update pose
  }

  // Check whether the solution is good

  return is_success;
}

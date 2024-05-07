#include <map>

#include "optimizer/pose_graph_optimizer_ceres_solver.h"
#include "optimizer/time_checker.h"

namespace optimizer {

PoseGraphOptimizerCeresSolver::PoseGraphOptimizerCeresSolver() {}

bool PoseGraphOptimizerCeresSolver::Solve(const Options& options,
                                          SummaryReporter* summary_reporter) {
  TimeChecker time_checker("PoseGraphOptimizerCeresSolver::Solve");
  const auto& max_iteration = options.iteration_handle.max_num_iterations;
  const auto& threshold_convergence_delta_error =
      options.convergence_handle.threshold_cost_change;
  const auto& threshold_convergence_delta_pose =
      options.convergence_handle.threshold_step_size;
  const auto& threshold_huber_loss =
      options.outlier_handle.threshold_huber_loss;
  const auto& threshold_outlier_reproj_error =
      options.outlier_handle.threshold_outlier_rejection;

  OverallSummary overall_summary;
  overall_summary.max_num_iterations = max_iteration;

  bool is_success = true;

  const auto num_constraints = constraint_list_.size();
  std::cerr << "num constraints; " << num_constraints;
  if (summary_reporter != nullptr) {
  }

  return is_success;
}

}  // namespace optimizer

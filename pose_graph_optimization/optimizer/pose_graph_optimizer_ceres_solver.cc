#include <map>

#include "optimizer/pose_graph_optimizer_ceres_solver.h"

namespace optimizer {

PoseGraphOptimizerCeresSolver::PoseGraphOptimizerCeresSolver() {}

bool PoseGraphOptimizerCeresSolver::Solve(const Options& options,
                                          SummaryReporter* summary_reporter) {
  bool is_solution_valid = true;

  (void)options;

  if (summary_reporter != nullptr) {
  }

  return is_solution_valid;
}

}  // namespace optimizer

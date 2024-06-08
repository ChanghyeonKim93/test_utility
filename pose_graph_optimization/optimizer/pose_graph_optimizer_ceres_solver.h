#ifndef OPTIMIZER_POSE_GRAPH_OPTIMIZER_CERES_SOLVER_H_
#define OPTIMIZER_POSE_GRAPH_OPTIMIZER_CERES_SOLVER_H_

#include "optimizer/options.h"
#include "optimizer/pose_graph_optimizer.h"
#include "optimizer/summary_reporter.h"
#include "optimizer/types.h"
#include "utility/geometry_helper.h"

#include "Eigen/Dense"
#include "ceres/autodiff_cost_function.h"
#include "ceres/rotation.h"

namespace optimizer {

class PoseGraphOptimizerCeresSolver : public PoseGraphOptimizer {
 private:
 public:
  PoseGraphOptimizerCeresSolver();
  bool Solve(const Options& options,
             SummaryReporter* summary_reporter = nullptr) final;
};

}  // namespace optimizer

#endif  // OPTIMIZER_POSE_GRAPH_OPTIMIZER_CERES_SOLVER_H_
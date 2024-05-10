#include "optimizer/pose_graph_optimizer_ceres_solver.h"

#include <map>

#include "ceres/ceres.h"
#include "ceres/problem.h"
#include "utility/time_checker.h"

namespace optimizer {

PoseGraphOptimizerCeresSolver::PoseGraphOptimizerCeresSolver() {}

bool PoseGraphOptimizerCeresSolver::Solve(const Options& options,
                                          SummaryReporter* summary_reporter) {
  utility::TimeChecker time_checker("PoseGraphOptimizerCeresSolver::Solve");
  const auto& iteration_handle = options.iteration_handle;
  const auto& convergence_handle = options.convergence_handle;

  //   const auto& outlier_handle = options.outlier_handle;
  //   const auto& threshold_huber_loss = outlier_handle.threshold_huber_loss;
  //   const auto& threshold_outlier_reproj_error =
  //       outlier_handle.threshold_outlier_rejection;

  OverallSummary overall_summary;
  overall_summary.max_num_iterations = iteration_handle.max_num_iterations;

  bool is_success = true;

  // Add constraints
  ceres::Problem ceres_problem;
  for (const auto& constraint : constraint_list_) {
    ceres::CostFunction* cost_function =
        RelativePoseCostFunctor::Create(constraint.relative_pose_01, {});

    const auto& pose_ptr0 = index_to_pose_ptr_map_.at(constraint.index0);
    const auto& pose_ptr1 = index_to_pose_ptr_map_.at(constraint.index1);
    ceres_problem.AddResidualBlock(
        cost_function, nullptr,
        pose_ptr_to_parameter_map_[pose_ptr0].position.data(),
        pose_ptr_to_parameter_map_[pose_ptr0].rvec.data(),
        pose_ptr_to_parameter_map_[pose_ptr1].position.data(),
        pose_ptr_to_parameter_map_[pose_ptr1].rvec.data());
  }

  // Fix poses
  for (auto& ptr : fixed_pose_ptr_set_) {
    ceres_problem.SetParameterBlockConstant(
        pose_ptr_to_parameter_map_.at(ptr).position.data());
    ceres_problem.SetParameterBlockConstant(
        pose_ptr_to_parameter_map_.at(ptr).rvec.data());
  }

  // Solve the problem
  ceres::Solver::Summary ceres_summary;
  ceres::Solver::Options ceres_options;
  ceres_options.max_num_iterations = iteration_handle.max_num_iterations;
  ceres_options.function_tolerance = convergence_handle.threshold_cost_change;
  ceres_options.parameter_tolerance = convergence_handle.threshold_step_size;
  //   ceres_options.minimizer_progress_to_stdout = true;
  ceres::Solve(ceres_options, &ceres_problem, &ceres_summary);

  if (summary_reporter != nullptr) {
    for (const auto& ceres_iteration_summary : ceres_summary.iterations) {
      IterationSummary iteration_summary;
      iteration_summary.cost = ceres_iteration_summary.cost;
      iteration_summary.cost_change = ceres_iteration_summary.cost_change;
      iteration_summary.cumulative_time_in_seconds =
          ceres_iteration_summary.cumulative_time_in_seconds;
      iteration_summary.gradient_norm = ceres_iteration_summary.gradient_norm;
      iteration_summary.iteration = ceres_iteration_summary.iteration;
      if (ceres_iteration_summary.step_is_successful)
        iteration_summary.iteration_status = IterationStatus::UPDATE;
      else
        iteration_summary.iteration_status = IterationStatus::SKIPPED;
      iteration_summary.iteration_time_in_seconds =
          ceres_iteration_summary.iteration_time_in_seconds;
      iteration_summary.step_norm = ceres_iteration_summary.step_norm;
      iteration_summary.step_size = ceres_iteration_summary.step_size;
      iteration_summary.step_solver_time_in_seconds =
          ceres_iteration_summary.step_solver_time_in_seconds;
      iteration_summary.trust_region_radius =
          ceres_iteration_summary.trust_region_radius;
      summary_reporter->SetIterationSummary(iteration_summary);
    }

    overall_summary.final_cost = ceres_summary.final_cost;
    overall_summary.initial_cost = ceres_summary.initial_cost;
    if (ceres_summary.termination_type == ceres::TerminationType::CONVERGENCE)
      overall_summary.is_converged = true;
    else
      overall_summary.is_converged = false;
    overall_summary.num_parameter_blocks = ceres_summary.num_parameter_blocks;
    overall_summary.num_parameters = ceres_summary.num_parameters;
    overall_summary.num_residual_blocks = ceres_summary.num_residual_blocks;
    overall_summary.num_residuals = ceres_summary.num_residuals;
    summary_reporter->SetOverallSummary(overall_summary);
  }

  // Check solver states
  if (ceres_summary.IsSolutionUsable()) {
    // Update poses
    for (auto& [pose_ptr, parameter] : pose_ptr_to_parameter_map_) {
      std::cerr << "IN!!: " << parameter.position.transpose() << std::endl;
      *pose_ptr = ConvertToPose(parameter);
    }
  } else {
    is_success = false;
  }

  return is_success;
}

}  // namespace optimizer

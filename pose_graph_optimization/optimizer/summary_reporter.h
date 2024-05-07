
#ifndef OPTIMIZER_SUMMARY_REPORTER_H_
#define OPTIMIZER_SUMMARY_REPORTER_H_

#include <string>
#include <vector>

namespace optimizer {

enum class IterationStatus {
  UNDEFINED = -1,
  UPDATE = 0,
  UPDATE_TRUST_MORE = 1,
  SKIPPED = 2
};

struct OverallSummary {
  double initial_cost{0.0};
  double final_cost{0.0};
  bool is_converged{true};
  int num_residuals{-1};
  int num_residual_blocks{-1};
  int num_parameters{-1};
  int num_parameter_blocks{-1};
};

struct IterationSummary {
  int iteration{-1};
  double iteration_time_in_seconds{0.0};
  double cumulative_time_in_seconds{0.0};
  double cost{0.0};
  double cost_change{0.0};
  double gradient_norm{0.0};
  double step_norm{0.0};
  double step_size{0.0};
  double step_solver_time_in_seconds{0.0};
  double trust_region_radius{0.0};
  IterationStatus iteration_status{IterationStatus::UNDEFINED};
};

class SummaryReporter {
 public:
  SummaryReporter();
  ~SummaryReporter();

  const std::string BriefReport() const;
  double GetTotalTimeInSeconds() const;

  void SetIterationSummary(const IterationSummary& iteration_summary);
  void SetOverallSummary(const OverallSummary& overall_summary);

 private:
  bool is_overall_summary_set_{false};
  bool is_iteration_summary_set_{false};

  OverallSummary overall_summary_;
  std::vector<IterationSummary> iteration_summary_list_;
};

}  // namespace optimizer

#endif  // OPTIMIZER_SUMMARY_REPORTER_H_
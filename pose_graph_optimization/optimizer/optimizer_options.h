#ifndef OPTIMIZER_OPTIMIZER_OPTIONS_H_
#define OPTIMIZER_OPTIMIZER_OPTIONS_H_

namespace optimizer {

struct Options {
  enum class SolverType {
    UNDEFINED = -1,
    GRADIENT_DESCENT = 0,
    GAUSS_NEWTON = 1,
    LEVENBERG_MARQUARDT = 2
  };

  SolverType solver_type{SolverType::GAUSS_NEWTON};
  struct {
    double threshold_step_size{1e-5};
    double threshold_cost_change{1e-5};
  } convergence_handle;
  struct {
    double threshold_huber_loss{1.0};
    double threshold_outlier_rejection{2.0};
  } outlier_handle;
  struct {
    int max_num_iterations{50};
  } iteration_handle;
  struct {
    double initial_lambda{100.0};
    double decrease_ratio_lambda{0.33f};
    double increase_ratio_lambda{3.0f};
  } trust_region_handle;
};

}  // namespace optimizer

#endif  // OPTIMIZER_OPTIMIZER_OPTIONS_H_
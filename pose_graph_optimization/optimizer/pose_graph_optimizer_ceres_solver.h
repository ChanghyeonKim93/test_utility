#ifndef OPTIMIZER_POSE_GRAPH_OPTIMIZER_CERES_SOLVER_H_
#define OPTIMIZER_POSE_GRAPH_OPTIMIZER_CERES_SOLVER_H_

#include <map>

#include "optimizer/options.h"
#include "optimizer/pose_graph_optimizer.h"
#include "optimizer/summary_reporter.h"
#include "optimizer/types.h"

#include "Eigen/Dense"
#include "ceres/autodiff_cost_function.h"

namespace optimizer {

class PoseGraphOptimizerCeresSolver : public PoseGraphOptimizer {
 private:
  class RelativePoseCostFunctor {
   public:
    RelativePoseCostFunctor(const Pose& T01) : T01_(T01) {}
    template <typename T>
    bool operator()(const T* const t0, const T* const w0, const T* const t1,
                    const T* const w1, T* residuals) const {
      residuals[0] = 1.0;
      residuals[1] = 1.0;
      residuals[2] = 1.0;
      residuals[3] = 1.0;
      residuals[4] = 1.0;
      residuals[5] = 1.0;

      return true;
    }

   private:
    Pose T01_;
  };

  class CostFunctor {
   public:
    CostFunctor(const Pose& relative_pose_measured_01)
        : relative_pose_measured_01_(relative_pose_measured_01) {}

    //    t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
    //           [ q_ab ]    [ q_a^{-1] * q_b         ]
    //
    //   error = [ p_ab - \hat{p}_ab                 ]
    //           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]

    template <typename T>
    bool operator()(const T* const p_a_ptr, const T* const q_a_ptr,
                    const T* const p_b_ptr, const T* const q_b_ptr,
                    T* residuals_ptr) const {
      Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
      Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

      Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
      Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

      // Compute the relative transformation between the two frames.
      Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
      Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

      // Represent the displacement between the two frames in the A frame.
      Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

      // Compute the error between the two orientation estimates.
      Eigen::Quaternion<T> delta_q =
          relative_pose_measured_01.q.template cast<T>() *
          q_ab_estimated.conjugate();

      // Compute the residuals.
      // [ position         ]   [ delta_p          ]
      // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
      Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
      residuals.template block<3, 1>(0, 0) =
          p_ab_estimated - t_ab_measured_.p.template cast<T>();
      residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

      // Scale the residuals by the measurement uncertainty.
      // residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

      return true;
    }

    static ceres::CostFunction* Create(
        const Pose& relative_pose_measured_01,
        const Eigen::Matrix<double, 6, 6>& sqrt_information) {
      (void)sqrt_information;
      return new ceres::AutoDiffCostFunction<CostFunctor, 6, 3, 4, 3, 4>(
          new CostFunctor(relative_pose_measured_01));
    }

   private:
    const Pose relative_pose_measured_01_;
    // The square root of the measurement information matrix.
  };

 public:
  PoseGraphOptimizerCeresSolver();
  bool Solve(const Options& options,
             SummaryReporter* summary_reporter = nullptr) final;
};

}  // namespace optimizer

#endif  // OPTIMIZER_POSE_GRAPH_OPTIMIZER_CERES_SOLVER_H_
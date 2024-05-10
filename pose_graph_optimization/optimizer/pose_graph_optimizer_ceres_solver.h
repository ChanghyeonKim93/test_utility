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
  class RelativePoseCostFunctor {
   public:
    RelativePoseCostFunctor(const Pose& relative_pose_01)
        : rel_pose_01_(relative_pose_01),
          rel_quat_01_(utility::GeometryHelper::ConvertToQuaternion(
              Eigen::Matrix3d(rel_pose_01_.linear()))) {}

    //    t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
    //           [ q_ab ]    [ q_a^{-1] * q_b         ]
    //
    //   error = [ p_ab - \hat{p}_ab                 ]
    //           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]

    template <typename T>
    bool operator()(const T* const pos0, const T* const rvec0,
                    const T* const pos1, const T* const rvec1,
                    T* residuals) const {
      Eigen::Map<const Eigen::Matrix<T, 3, 1>> t0(pos0);
      Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(pos1);

      T q0_tmp[4];
      T q1_tmp[4];
      ceres::AngleAxisToQuaternion(rvec0, q0_tmp);
      ceres::AngleAxisToQuaternion(rvec1, q1_tmp);

      Eigen::Map<const Eigen::Quaternion<T>> q0(q0_tmp);
      Eigen::Map<const Eigen::Quaternion<T>> q1(q1_tmp);
      Eigen::Quaternion<T> q0_inv = q0.conjugate();

      Eigen::Quaternion<T> error_q =
          rel_quat_01_.template cast<T>().conjugate() * (q0_inv * q1);

      Eigen::Map<Eigen::Matrix<T, 6, 1>> residual_vector(residuals);
      residual_vector.template block<3, 1>(0, 0) =
          q0 * rel_pose_01_.translation().template cast<T>() - (t1 - t0);
      residual_vector.template block<3, 1>(3, 0) = T(2.0) * error_q.vec();

      // // Scale the residuals by the measurement uncertainty.
      // // residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

      return true;
    }

    static ceres::CostFunction* Create(
        const Pose& relative_pose_01,
        const Eigen::Matrix<double, 6, 6>& sqrt_information) {
      (void)sqrt_information;
      return new ceres::AutoDiffCostFunction<RelativePoseCostFunctor, 6, 3, 3,
                                             3, 3>(
          new RelativePoseCostFunctor(relative_pose_01));
    }

   private:
    Pose rel_pose_01_;
    Eigen::Quaterniond rel_quat_01_;
    // The square root of the measurement information matrix.
  };

 public:
  PoseGraphOptimizerCeresSolver();
  bool Solve(const Options& options,
             SummaryReporter* summary_reporter = nullptr) final;
};

}  // namespace optimizer

#endif  // OPTIMIZER_POSE_GRAPH_OPTIMIZER_CERES_SOLVER_H_
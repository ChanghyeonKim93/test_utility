#ifndef OPTIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_CERES_COST_FUNCTOR_H_
#define OPTIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_CERES_COST_FUNCTOR_H_

#include "types.h"

#include "Eigen/Dense"
#include "ceres/ceres.h"

namespace optimizer {
namespace mahalanobis_distance_minimizer {

class RedundantMahalanobisDistanceCostFunctor {
 public:
  RedundantMahalanobisDistanceCostFunctor(
      const std::vector<Correspondence>& correspondences, const double c1,
      const double c2)
      : c1_(c1), c2_(c2) {
    correspondences_ = correspondences;
  }

  template <typename T>
  bool operator()(const T* const translation_ptr, const T* const quaternion_ptr,
                  T* residual) const {
    Eigen::Matrix<T, 3, 1> translation(translation_ptr[0], translation_ptr[1],
                                       translation_ptr[2]);
    Eigen::Quaternion<T> rotation(quaternion_ptr[0], quaternion_ptr[1],
                                  quaternion_ptr[2], quaternion_ptr[3]);
    for (size_t i = 0; i < correspondences_.size(); ++i) {
      const auto& corr = correspondences_[i];
      Eigen::Matrix<T, 3, 1> warped_point =
          rotation * corr.point.cast<T>() + translation;
      Eigen::Matrix<T, 3, 1> e_i = warped_point - corr.ndt.mean.cast<T>();
      T squared_mahalanobis_dist =
          e_i.transpose() * corr.ndt.information.cast<T>() * e_i;
      residual[i] = c1_ - c1_ * ceres::exp(-c2_ * squared_mahalanobis_dist);
    }
    return true;
  }

  static ceres::CostFunction* Create(
      const std::vector<Correspondence>& correspondences, const double c1,
      const double c2) {
    return new ceres::AutoDiffCostFunction<
        RedundantMahalanobisDistanceCostFunctor, ceres::DYNAMIC, 3, 4>(
        new RedundantMahalanobisDistanceCostFunctor(correspondences, c1, c2),
        correspondences.size());
  }

 private:
  std::vector<Correspondence> correspondences_;
  const double c1_;
  const double c2_;
};

class MahalanobisDistanceCostFunctor {
 public:
  MahalanobisDistanceCostFunctor(const Correspondence& correspondence) {
    correspondence_ = correspondence;
  }

  template <typename T>
  bool operator()(const T* const translation_ptr, const T* const quaternion_ptr,
                  T* residual) const {
    Eigen::Matrix<T, 3, 1> translation(translation_ptr[0], translation_ptr[1],
                                       translation_ptr[2]);
    Eigen::Quaternion<T> rotation(quaternion_ptr[3], quaternion_ptr[0],
                                  quaternion_ptr[1], quaternion_ptr[2]);
    Eigen::Matrix<T, 3, 1> warped_point =
        rotation * correspondence_.point.cast<T>() + translation;
    Eigen::Matrix<T, 3, 1> e_i =
        warped_point - correspondence_.ndt.mean.cast<T>();
    Eigen::Matrix<T, 3, 1> r =
        correspondence_.ndt.sqrt_information.cast<T>() * e_i;
    residual[0] = r(0);
    residual[1] = r(1);
    residual[2] = r(2);
    return true;
  }

  static ceres::CostFunction* Create(const Correspondence& correspondence) {
    return new ceres::AutoDiffCostFunction<MahalanobisDistanceCostFunctor, 3, 3,
                                           4>(
        new MahalanobisDistanceCostFunctor(correspondence));
  }

 private:
  Correspondence correspondence_;
};

class CERES_EXPORT ExponentialLoss : public ceres::LossFunction {
 public:
  explicit ExponentialLoss(double c1, double c2)
      : c1_(c1), c2_(c2), two_c1c2_(2.0 * c1_ * c2_) {}

  void Evaluate(double squared_residual, double output[3]) const {
    const double exp_term = std::exp(-c2_ * squared_residual);
    output[0] = c1_ - c1_ * exp_term;
    output[1] = two_c1c2_ * output[0];
    output[2] = -2.0 * c2_ * output[1];
  }

 private:
  const double c1_;
  const double c2_;
  const double two_c1c2_;
};

}  // namespace mahalanobis_distance_minimizer
}  // namespace optimizer

#endif  // OPTIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_CERES_COST_FUNCTOR_H_
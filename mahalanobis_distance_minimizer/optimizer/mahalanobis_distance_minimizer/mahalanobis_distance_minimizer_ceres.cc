#include "optimizer/mahalanobis_distance_minimizer/mahalanobis_distance_minimizer_ceres.h"

#include "ceres/ceres.h"

#include "optimizer/mahalanobis_distance_minimizer/ceres_cost_functor.h"

namespace optimizer {
namespace mahalanobis_distance_minimizer {

MahalanobisDistanceMinimizerCeres::MahalanobisDistanceMinimizerCeres() {}

MahalanobisDistanceMinimizerCeres::~MahalanobisDistanceMinimizerCeres() {}

bool MahalanobisDistanceMinimizerCeres::Solve(
    const std::vector<Correspondence>& correspondences, Pose* pose) {
  constexpr double kC1{1.0};
  constexpr double kC2{1.0};

  const Pose initial_pose = *pose;

  Eigen::Vector3d optimized_translation(initial_pose.translation().x(),
                                        initial_pose.translation().y(),
                                        initial_pose.translation().z());
  Orientation optimized_orientation(initial_pose.rotation());

  ceres::LossFunction* loss_function = new ExponentialLoss(kC1, kC2);
  ceres::Problem problem;
  for (size_t index = 0; index < correspondences.size(); ++index) {
    ceres::CostFunction* cost_function =
        MahalanobisDistanceCostFunctor::Create(correspondences.at(index));
    problem.AddResidualBlock(cost_function, loss_function,
                             optimized_translation.data(),
                             optimized_orientation.coeffs().data());
  }
  ceres::LocalParameterization* quaternion_parameterization =
      new ceres::EigenQuaternionParameterization();
  problem.SetParameterization(optimized_orientation.coeffs().data(),
                              quaternion_parameterization);
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cerr << "Summary:\n";
  std::cerr << summary.BriefReport() << std::endl;

  return true;
}

bool MahalanobisDistanceMinimizerCeres::SolveByRedundant(
    const std::vector<Correspondence>& correspondences, Pose* pose) {
  constexpr double kC1{1.0};
  constexpr double kC2{1.0};

  ceres::Problem problem;
  ceres::CostFunction* cost_function =
      RedundantMahalanobisDistanceCostFunctor::Create(correspondences, kC1,
                                                      kC2);

  return true;
}

}  // namespace mahalanobis_distance_minimizer
}  // namespace optimizer
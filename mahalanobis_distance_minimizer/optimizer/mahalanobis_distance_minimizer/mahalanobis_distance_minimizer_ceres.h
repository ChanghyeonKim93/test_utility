#ifndef OPTIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_CERES_H_
#define OPTIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_CERES_H_

#include <vector>

#include "optimizer/mahalanobis_distance_minimizer/mahalanobis_distance_minimizer.h"
#include "types.h"

namespace optimizer {
namespace mahalanobis_distance_minimizer {

class MahalanobisDistanceMinimizerCeres : public MahalanobisDistanceMinimizer {
 public:
  MahalanobisDistanceMinimizerCeres();

  ~MahalanobisDistanceMinimizerCeres();

  bool Solve(const std::vector<Correspondence>& correspondences,
             Pose* pose) final;

  bool SolveByRedundant(const std::vector<Correspondence>& correspondences,
                        Pose* pose);
};

}  // namespace mahalanobis_distance_minimizer
}  // namespace optimizer

#endif  // OPTIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_CERES_H_
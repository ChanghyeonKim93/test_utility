#ifndef OPTIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_H_
#define OPTIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_H_

#include <memory>

#include "optimizer/loss_function.h"
#include "types.h"

namespace optimizer {
namespace mahalanobis_distance_minimizer {

class MahalanobisDistanceMinimizer {
 public:
  MahalanobisDistanceMinimizer() {}

  ~MahalanobisDistanceMinimizer() {}

  void SetLossFunction(const std::shared_ptr<LossFunction>& loss_function) {
    loss_function_ = loss_function;
  }

  virtual bool Solve(const std::vector<Correspondence>& correspondences,
                     Pose* pose) = 0;

 private:
  std::shared_ptr<LossFunction> loss_function_{nullptr};
};

}  // namespace mahalanobis_distance_minimizer
}  // namespace optimizer

#endif  // OPTIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_MAHALANOBIS_DISTANCE_MINIMIZER_H_
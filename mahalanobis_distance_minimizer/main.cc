#include <iostream>
#include <memory>

#include "Eigen/Dense"

#include "optimizer/mahalanobis_distance_minimizer/mahalanobis_distance_minimizer.h"
#include "optimizer/mahalanobis_distance_minimizer/mahalanobis_distance_minimizer_ceres.h"
#include "time_checker.h"
#include "types.h"

int main(int argc, char** argv) {
  std::unique_ptr<
      optimizer::mahalanobis_distance_minimizer::MahalanobisDistanceMinimizer>
      optimizer = std::make_unique<optimizer::mahalanobis_distance_minimizer::
                                       MahalanobisDistanceMinimizerCeres>();

  return 0;
}
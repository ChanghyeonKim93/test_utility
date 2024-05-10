#ifndef OPTIMIZER_POSE_GRAPH_OPTIMIZER_H_
#define OPTIMIZER_POSE_GRAPH_OPTIMIZER_H_

#include <unordered_map>
#include <unordered_set>

#include "optimizer/options.h"
#include "optimizer/summary_reporter.h"
#include "optimizer/types.h"

namespace optimizer {

using LieAlgebra = Eigen::Matrix<double, 6, 1>;
struct PositionAndRotationVector {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rvec{Eigen::Vector3d::Zero()};
};

class PoseGraphOptimizer {
 public:
  PoseGraphOptimizer();

  void RegisterPose(const int index, Pose* pose_ptr);
  void MakePoseFixed(Pose* original_pose);
  void AddConstraint(const RelativePoseConstraint& constraint);
  void Reset();

  virtual bool Solve(const Options& options,
                     SummaryReporter* summary_reporter = nullptr) = 0;

 protected:
  // std::unordered_map<Pose*, Pose> original_pose_to_opt_pose_map_;  // a->c
  std::unordered_map<Pose*, PositionAndRotationVector>
      pose_ptr_to_parameter_map_;                         // a->c
  std::unordered_map<int, Pose*> index_to_pose_ptr_map_;  // b->a
  std::unordered_map<Pose*, int> pose_ptr_to_index_map_;  // a->b

  std::unordered_set<Pose*> fixed_pose_ptr_set_;

  std::vector<RelativePoseConstraint> constraint_list_;

  const PositionAndRotationVector ConvertToPositionAndRotationVector(
      const Pose& pose);
  const Pose ConvertToPose(const PositionAndRotationVector& pos_and_rvec);
};

}  // namespace optimizer

#endif  // OPTIMIZER_POSE_GRAPH_OPTIMIZER_H_
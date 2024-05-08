#ifndef OPTIMIZER_POSE_GRAPH_OPTIMIZER_H_
#define OPTIMIZER_POSE_GRAPH_OPTIMIZER_H_

#include <unordered_map>
#include <unordered_set>

#include "optimizer/options.h"
#include "optimizer/summary_reporter.h"
#include "optimizer/types.h"

namespace optimizer {

class PoseGraphOptimizer {
 public:
  PoseGraphOptimizer();

  void RegisterPose(const int index, Pose* pose_ptr);
  void MakePoseFixed(Pose* original_pose);
  void AddConstraint(const Constraint& constraint);
  void Reset();

  virtual bool Solve(const Options& options,
                     SummaryReporter* summary_reporter = nullptr) = 0;

 protected:
  std::unordered_map<Pose*, Pose> original_pose_to_opt_pose_map_;  // a->c
  std::unordered_map<int, Pose*> index_to_original_pose_map_;      // b->a
  std::unordered_map<Pose*, int> original_pose_to_index_map_;      // a->b
  std::unordered_set<Pose*> fixed_original_pose_set_;
  std::vector<Constraint> constraint_list_;
};

}  // namespace optimizer

#endif  // OPTIMIZER_POSE_GRAPH_OPTIMIZER_H_
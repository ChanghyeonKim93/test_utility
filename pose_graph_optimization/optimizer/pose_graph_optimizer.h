#ifndef OPTIMIZER_POSE_GRAPH_OPTIMIZER_H_
#define OPTIMIZER_POSE_GRAPH_OPTIMIZER_H_

#include <unordered_map>
#include <unordered_set>

#include "optimizer/optimizer_options.h"
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

 private:
  std::unordered_map<int, Pose*> index_to_pose_map_;
  std::unordered_map<Pose*, int> pose_to_index_map_;
  std::unordered_set<Pose*> fixed_pose_set_;
  std::vector<Constraint> constraint_list_;
};

}  // namespace optimizer

#endif  // OPTIMIZER_POSE_GRAPH_OPTIMIZER_H_
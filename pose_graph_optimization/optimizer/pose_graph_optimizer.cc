#include "optimizer/pose_graph_optimizer.h"

namespace optimizer {

PoseGraphOptimizer::PoseGraphOptimizer() {}

void PoseGraphOptimizer::RegisterPose(const int index, Pose* pose_ptr) {
  if (index_to_pose_map_.find(index) != index_to_pose_map_.end())
    throw std::runtime_error("Duplicated pose index");
  if (pose_ptr == nullptr) throw std::runtime_error("null pose ptr");
  if (pose_to_index_map_.find(pose_ptr) != pose_to_index_map_.end())
    throw std::runtime_error("Duplicated pose_ptr");

  index_to_pose_map_.insert({index, pose_ptr});
  pose_to_index_map_.insert({pose_ptr, index});
}

void PoseGraphOptimizer::MakePoseFixed(Pose* original_pose) {
  if (original_pose == nullptr) throw std::runtime_error("null pose ptr");
  fixed_pose_set_.insert(original_pose);
}

void PoseGraphOptimizer::AddConstraint(const Constraint& constraint) {
  if (index_to_pose_map_.find(constraint.index0) == index_to_pose_map_.end())
    throw std::runtime_error("index0 does not exist.");
  if (index_to_pose_map_.find(constraint.index1) == index_to_pose_map_.end())
    throw std::runtime_error("index1 does not exist.");

  constraint_list_.push_back(constraint);
}

void PoseGraphOptimizer::Reset() {
  pose_to_index_map_.clear();
  index_to_pose_map_.clear();
  fixed_pose_set_.clear();
  constraint_list_.clear();
}

}  // namespace optimizer

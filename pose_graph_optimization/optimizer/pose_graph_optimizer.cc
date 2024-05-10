#include "optimizer/pose_graph_optimizer.h"

#include "utility/geometry_helper.h"

namespace optimizer {

PoseGraphOptimizer::PoseGraphOptimizer() {}

void PoseGraphOptimizer::RegisterPose(const int index, Pose* pose_ptr) {
  if (index_to_pose_ptr_map_.find(index) != index_to_pose_ptr_map_.end())
    throw std::runtime_error("Duplicated pose index");
  if (pose_ptr == nullptr) throw std::runtime_error("null pose ptr");
  if (pose_ptr_to_index_map_.find(pose_ptr) != pose_ptr_to_index_map_.end())
    throw std::runtime_error("Duplicated pose_ptr");

  index_to_pose_ptr_map_.insert({index, pose_ptr});
  pose_ptr_to_index_map_.insert({pose_ptr, index});
  pose_ptr_to_parameter_map_.insert(
      {pose_ptr, ConvertToPositionAndRotationVector(*pose_ptr)});
}

void PoseGraphOptimizer::MakePoseFixed(Pose* pose_ptr) {
  if (pose_ptr == nullptr) throw std::runtime_error("null pose ptr");
  fixed_pose_ptr_set_.insert(pose_ptr);
}

void PoseGraphOptimizer::AddConstraint(
    const RelativePoseConstraint& constraint) {
  if (index_to_pose_ptr_map_.find(constraint.index0) ==
      index_to_pose_ptr_map_.end())
    throw std::runtime_error("index0 does not exist.");
  if (index_to_pose_ptr_map_.find(constraint.index1) ==
      index_to_pose_ptr_map_.end())
    throw std::runtime_error("index1 does not exist.");

  constraint_list_.push_back(constraint);
}

void PoseGraphOptimizer::Reset() {
  pose_ptr_to_parameter_map_.clear();
  pose_ptr_to_index_map_.clear();
  index_to_pose_ptr_map_.clear();
  fixed_pose_ptr_set_.clear();
  constraint_list_.clear();
}

const PositionAndRotationVector
PoseGraphOptimizer::ConvertToPositionAndRotationVector(const Pose& pose) {
  PositionAndRotationVector pos_and_rvec;
  pos_and_rvec.position = pose.translation();
  pos_and_rvec.rvec =
      utility::GeometryHelper::ConvertToRotationVector(pose.linear());
  return pos_and_rvec;
}

const Pose PoseGraphOptimizer::ConvertToPose(
    const PositionAndRotationVector& pos_and_rvec) {
  Pose pose{Pose::Identity()};
  pose.translation() = pos_and_rvec.position;
  pose.linear() =
      utility::GeometryHelper::ConvertToQuaternion(pos_and_rvec.rvec)
          .toRotationMatrix();
  return pose;
}

}  // namespace optimizer

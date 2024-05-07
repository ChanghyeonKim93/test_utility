#include <iostream>
#include <memory>

#include "optimizer/pose_graph_optimizer.h"
#include "optimizer/pose_graph_optimizer_ceres_solver.h"
#include "optimizer/types.h"

int main() {
  std::unique_ptr<optimizer::PoseGraphOptimizer> optimizer;
  optimizer = std::make_unique<optimizer::PoseGraphOptimizerCeresSolver>();

  optimizer::Pose del_pose;
  del_pose.translation() = Eigen::Vector3d{0.01, -0.05, 0};
  del_pose.linear() = Eigen::Quaterniond{1, 0, 0, 0}.toRotationMatrix();

  optimizer::Pose pose0;
  pose0.translation() = Eigen::Vector3d{0, 0, 0};
  pose0.linear() = Eigen::Quaterniond{1, 0, 0, 0}.toRotationMatrix();

  optimizer::Pose pose1;
  pose1.translation() = Eigen::Vector3d{0, 1, 0};
  pose1.linear() = Eigen::Quaterniond{1, 0, 0, 0}.toRotationMatrix();

  optimizer::Constraint constraint;
  constraint.index0 = 0;
  constraint.index1 = 1;
  constraint.relative_pose_01 = pose0.inverse() * pose1;
  constraint.type = optimizer::Constraint::Type::kOdometry;

  std::vector<optimizer::Pose> pose_list;
  pose_list.push_back(pose0);
  pose_list.push_back(pose1 * del_pose.inverse());

  optimizer->RegisterPose(0, &pose_list[0]);
  optimizer->RegisterPose(1, &pose_list[1]);
  optimizer->AddConstraint(constraint);
  optimizer->MakePoseFixed(&pose_list[0]);

  optimizer::Options options;
  options.iteration_handle.max_num_iterations = 100;
  optimizer::SummaryReporter summary_reporter;

  optimizer->Solve(options, &summary_reporter);

  std::cerr << summary_reporter.BriefReport() << std::endl;

  return 0;
}
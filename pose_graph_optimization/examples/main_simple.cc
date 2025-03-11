#include <iostream>
#include <memory>

#include "optimizer/pose_graph_optimizer.h"
#include "optimizer/pose_graph_optimizer_ceres_solver.h"
#include "optimizer/types.h"

int main() {
  std::unique_ptr<optimizer::PoseGraphOptimizer> optimizer;
  optimizer = std::make_unique<optimizer::PoseGraphOptimizerCeresSolver>();

  optimizer::Pose del_pose;
  del_pose.translation() = Eigen::Vector3d{0.1, 0.25, 0};
  del_pose.linear() = utility::GeometryHelper::ConvertToRotationMatrix(
      Eigen::Vector3d(-0.04, 0.02, 0.1));

  optimizer::Pose pose0;
  pose0.translation() = Eigen::Vector3d{0, 0, 0};
  pose0.linear() = utility::GeometryHelper::ConvertToRotationMatrix(
      Eigen::Vector3d(0, 0, 0));

  optimizer::Pose pose1;
  pose1.translation() = Eigen::Vector3d{0, 0.2, 0};
  pose1.linear() = utility::GeometryHelper::ConvertToRotationMatrix(
      Eigen::Vector3d(0, 0, 0.1));

  optimizer::Pose pose2;
  pose2.translation() = Eigen::Vector3d{0.2, 0.5, 0};
  pose2.linear() = utility::GeometryHelper::ConvertToRotationMatrix(
      Eigen::Vector3d(0, 0, 0.2));

  optimizer::Pose pose3;
  pose3.translation() = Eigen::Vector3d{0.1, 0.1, 0};
  pose3.linear() = utility::GeometryHelper::ConvertToRotationMatrix(
      Eigen::Vector3d(0, 0, 0.1));

  optimizer::Pose pose4;
  pose4.translation() = Eigen::Vector3d{0.1, 0.05, 0};
  pose4.linear() = utility::GeometryHelper::ConvertToRotationMatrix(
      Eigen::Vector3d(0, 0, 0));

  optimizer::RelativePoseConstraint constraint0{
      optimizer::RelativePoseConstraint::Type::kOdometry, 0, 1,
      pose0.inverse() * pose1};
  optimizer::RelativePoseConstraint constraint1{
      optimizer::RelativePoseConstraint::Type::kOdometry, 1, 2,
      pose1.inverse() * pose2};
  optimizer::RelativePoseConstraint constraint2{
      optimizer::RelativePoseConstraint::Type::kOdometry, 2, 3,
      pose2.inverse() * pose3};
  optimizer::RelativePoseConstraint constraint3{
      optimizer::RelativePoseConstraint::Type::kOdometry, 3, 4,
      pose3.inverse() * pose4};
  optimizer::RelativePoseConstraint constraint4{
      optimizer::RelativePoseConstraint::Type::kLoop, 4, 0,
      pose4.inverse() * pose0};

  std::vector<optimizer::Pose> true_pose_list;
  true_pose_list.push_back(pose0);
  true_pose_list.push_back(pose1);
  true_pose_list.push_back(pose2);
  true_pose_list.push_back(pose3);
  true_pose_list.push_back(pose4);

  std::vector<optimizer::Pose> pose_list;
  pose_list.push_back(pose0);
  pose_list.push_back(pose1 * del_pose.inverse());
  pose_list.push_back(pose2 * del_pose);
  pose_list.push_back(pose3 * del_pose.inverse());
  pose_list.push_back(pose4 * del_pose * del_pose);

  optimizer->RegisterPose(0, &pose_list[0]);
  optimizer->RegisterPose(1, &pose_list[1]);
  optimizer->RegisterPose(2, &pose_list[2]);
  optimizer->RegisterPose(3, &pose_list[3]);
  optimizer->RegisterPose(4, &pose_list[4]);

  optimizer->AddConstraint(constraint0);
  optimizer->AddConstraint(constraint1);
  optimizer->AddConstraint(constraint2);
  optimizer->AddConstraint(constraint3);
  optimizer->AddConstraint(constraint4);

  optimizer->MakePoseFixed(&pose_list[0]);

  optimizer::Options options;
  options.iteration_handle.max_num_iterations = 100;
  optimizer::SummaryReporter summary_reporter;

  optimizer->Solve(options, &summary_reporter);
  std::cerr << summary_reporter.BriefReport() << std::endl;

  // Check result
  for (size_t i = 0; i < true_pose_list.size(); ++i) {
    std::cerr << "trans: " << true_pose_list[i].translation().transpose()
              << " / " << pose_list[i].translation().transpose() << "\n";
    std::cerr << "rvec: "
              << utility::GeometryHelper::ConvertToRotationVector(
                     Eigen::Matrix3d(true_pose_list[i].linear()))
                     .transpose()
              << " / "
              << utility::GeometryHelper::ConvertToRotationVector(
                     Eigen::Matrix3d(pose_list[i].linear()))
                     .transpose()
              << std::endl;
  }

  return 0;
}
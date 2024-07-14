#include <iostream>
#include <vector>

#include "Eigen/Dense"

#include "se3.h"

int main() {
  SE3 infinitisimal_pose(Eigen::Quaterniond(1, 0, 0, 0.001),
                         Eigen::Vector3d(0.0001, 0.0002, 0.0));
  SE3 pose(Eigen::Quaterniond(1, 2, 3, 4), Eigen::Vector3d(1, 2, 3));

  std::cerr << (pose * infinitisimal_pose).translation().transpose()
            << std::endl;

  SE3 inv_pose = pose.inverse();
  std::cerr << pose.rotation().convertToRotationMatrix() << std::endl;
  std::cerr << inv_pose.rotation().convertToRotationMatrix() << std::endl;
  std::cerr << inv_pose.translation().transpose() << std::endl;

  SE3 iden = pose * inv_pose;
  std::cerr << iden.rotation().convertToRotationMatrix() << std::endl;
  return 0;
}
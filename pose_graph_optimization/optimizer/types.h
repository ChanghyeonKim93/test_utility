#ifndef OPTIMIZER_TYPES_H_
#define OPTIMIZER_TYPES_H_

#include "Eigen/Dense"

namespace optimizer {

using Pose = Eigen::Transform<double, 3, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Mat33 = Eigen::Matrix<double, 3, 3>;
using Quaternion = Eigen::Quaternion<double>;

struct Constraint {
  enum class Type { kOdometry = 0, kLoop };
  int index0;
  int index1;
  Type type;
  Pose relative_pose_01;
};

}  // namespace optimizer

#endif  // OPTIMIZER_TYPES_H_
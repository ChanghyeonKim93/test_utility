#ifndef OPTIMIZER_TYPES_H_
#define OPTIMIZER_TYPES_H_

#include "Eigen/Dense"

namespace optimizer {

using Pose = Eigen::Transform<double, 3, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Mat33 = Eigen::Matrix<double, 3, 3>;
using Quaternion = Eigen::Quaternion<double>;

struct RelativePoseConstraint {
  enum class Type : uint8_t { kOdometry = 0, kLoop };
  Type type;
  int index0;
  int index1;
  Pose relative_pose_01;
};

struct AbsolutePoseConstraint {
  enum class Type : uint8_t { kGps = 0 };
  Type type;
  int index;
  Pose pose;
};

}  // namespace optimizer

#endif  // OPTIMIZER_TYPES_H_
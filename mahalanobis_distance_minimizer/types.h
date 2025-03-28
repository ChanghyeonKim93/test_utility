#ifndef TYPES_H_
#define TYPES_H_

#include <unordered_map>

#include "Eigen/Dense"

using Vec3 = Eigen::Vector3d;
using Mat3x3 = Eigen::Matrix3d;

using Orientation = Eigen::Quaterniond;
using Pose = Eigen::Isometry3d;

struct NDT {
  int count{0};
  Vec3 sum{Vec3::Zero()};
  Mat3x3 moment{Mat3x3::Identity()};

  Vec3 mean{Vec3::Zero()};
  Mat3x3 covariance{Mat3x3::Identity()};
  Mat3x3 information{Mat3x3::Identity()};
  Mat3x3 sqrt_information{Mat3x3::Identity()};
  bool is_valid{false};
  bool is_planar{false};
};

struct Correspondence {
  Vec3 point{Vec3::Zero()};
  NDT ndt;
};

#endif  // TYPES_H_
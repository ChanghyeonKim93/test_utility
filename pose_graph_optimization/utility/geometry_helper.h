#ifndef UTILITY_GEOMETRY_HELPER_H_
#define UTILITY_GEOMETRY_HELPER_H_

#include <cmath>

#include "Eigen/Dense"

namespace utility {

// refer to rotation.h in ceres
class GeometryHelper {
 public:
  // static const Eigen::Quaterniond& ConvertToQuaternion(const double roll,
  //                                                      const double pitch,
  //                                                      const double yaw);
  static const Eigen::Quaterniond& ConvertToQuaternion(
      const Eigen::Vector3d& rotation_vector);  //
  // static const Eigen::Quaterniond& ConvertToQuaternion(
  //     const Eigen::Matrix3d& rotation_matrix);

  // static const Eigen::Vector3d& ConvertToRotationVector(const double roll,
  //                                                       const double pitch,
  //                                                       const double yaw);
  // static const Eigen::Vector3d& ConvertToRotationVector(
  //     const Eigen::Matrix3d& rotation_matrix);
  static const Eigen::Vector3d& ConvertToRotationVector(
      const Eigen::Quaterniond& quaternion);

  static const Eigen::Matrix3d& ConvertToRotationMatrix(const double roll,
                                                        const double pitch,
                                                        const double yaw);
  static const Eigen::Matrix3d& ConvertToRotationMatrix(
      const Eigen::Vector3d& rotation_vector);
  // static const Eigen::Matrix3d& ConvertToRotationMatrix(
  //     const Eigen::Matrix3d& rotation_matrix);
};

}  // namespace utility

#endif  // UTILITY_GEOMETRY_HELPER_H_
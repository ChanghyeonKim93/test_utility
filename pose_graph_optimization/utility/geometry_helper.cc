#include "utility/geometry_helper.h"

namespace utility {

const Eigen::Quaterniond& GeometryHelper::ConvertToQuaternion(
    const Eigen::Vector3d& rotation_vector) {
  Eigen::Quaterniond quaternion{Eigen::Quaterniond::Identity()};
  const double ux = rotation_vector.x();
  const double uy = rotation_vector.y();
  const double uz = rotation_vector.z();
  const double theta = std::sqrt(ux * ux + uy * uy + uz * uz);

  if (theta > 0.0) {
    const double half_theta = theta * 0.5;
    const double k = std::sin(half_theta) / theta;
    quaternion.w() = std::cos(half_theta);
    quaternion.x() = ux * k;
    quaternion.y() = uy * k;
    quaternion.z() = uz * k;
  } else {
    quaternion.w() = 1.0;
    quaternion.x() = 0.0;
    quaternion.y() = 0.0;
    quaternion.z() = 0.0;
  }
  return quaternion;
}

const Eigen::Vector3d& GeometryHelper::ConvertToRotationVector(
    const Eigen::Quaterniond& quaternion) {
  Eigen::Vector3d rotation_vector;
  const double qx = quaternion.x();
  const double qy = quaternion.y();
  const double qz = quaternion.z();
  const double squared_sin_theta = qx * qx + qy * qy + qz * qz;

  if (squared_sin_theta > 0.0) {
    const double sin_theta = std::sqrt(squared_sin_theta);
    const double cos_theta = quaternion.w();

    const double two_theta =
        2.0 * ((cos_theta < 0.0) ? std::atan2(-sin_theta, -cos_theta)
                                 : std::atan2(sin_theta, cos_theta));
    const double k = two_theta / sin_theta;
    rotation_vector.x() = qx * k;
    rotation_vector.y() = qy * k;
    rotation_vector.z() = qz * k;
  } else {
    const double k = 2.0;
    rotation_vector.x() = qx * k;
    rotation_vector.y() = qy * k;
    rotation_vector.z() = qz * k;
  }
  return rotation_vector;
}

const Eigen::Matrix3d& GeometryHelper::ConvertToRotationMatrix(
    const double roll, const double pitch, const double yaw) {
  const double c1 = std::cos(yaw);
  const double s1 = std::sin(yaw);
  const double c2 = std::cos(roll);
  const double s2 = std::sin(roll);
  const double c3 = std::cos(pitch);
  const double s3 = std::sin(pitch);

  Eigen::Matrix3d rotation_matrix{Eigen::Matrix3d::Identity()};

  rotation_matrix(0, 0) = c1 * c2;
  rotation_matrix(0, 1) = -s1 * c3 + c1 * s2 * s3;
  rotation_matrix(0, 2) = s1 * s3 + c1 * s2 * c3;

  rotation_matrix(1, 0) = s1 * c2;
  rotation_matrix(1, 1) = c1 * c3 + s1 * s2 * s3;
  rotation_matrix(1, 2) = -c1 * s3 + s1 * s2 * c3;

  rotation_matrix(2, 0) = -s2;
  rotation_matrix(2, 1) = c2 * s3;
  rotation_matrix(2, 2) = c2 * c3;

  return rotation_matrix;
}

const Eigen::Matrix3d& GeometryHelper::ConvertToRotationMatrix(
    const Eigen::Vector3d& rotation_vector) {
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  const double theta = rotation_vector.norm();
  if (theta > std::numeric_limits<double>::epsilon()) {
    // We want to be careful to only evaluate the square root if the
    // norm of the angle_axis vector is greater than zero. Otherwise
    // we get a division by zero.
    const double wx = rotation_vector.x() / theta;
    const double wy = rotation_vector.y() / theta;
    const double wz = rotation_vector.z() / theta;

    const double costheta = std::cos(theta);
    const double sintheta = std::sin(theta);

    // clang-format off
    R(0, 0) =     costheta   + wx*wx*(1.0 -    costheta);
    R(1, 0) =  wz*sintheta   + wx*wy*(1.0 -    costheta);
    R(2, 0) = -wy*sintheta   + wx*wz*(1.0 -    costheta);
    R(0, 1) = -wz*sintheta   + wx*wy*(1.0 -    costheta);
    R(1, 1) =     costheta   + wy*wy*(1.0 -    costheta);
    R(2, 1) =  wx*sintheta   + wy*wz*(1.0 -    costheta);
    R(0, 2) =  wy*sintheta   + wx*wz*(1.0 -    costheta);
    R(1, 2) = -wx*sintheta   + wy*wz*(1.0 -    costheta);
    R(2, 2) =     costheta   + wz*wz*(1.0 -    costheta);
    // clang-format on
  } else {
    // Near zero, we switch to using the first order Taylor expansion.
    // clang-format off
    R(0, 0) =  1.0;
    R(1, 0) =  rotation_vector.z();
    R(2, 0) = -rotation_vector.y();
    R(0, 1) = -rotation_vector.z();
    R(1, 1) =  1.0;
    R(2, 1) =  rotation_vector.x();
    R(0, 2) =  rotation_vector.y();
    R(1, 2) = -rotation_vector.x();
    R(2, 2) =  1.0;
    // clang-format on
  }

  return R;
}

}  // namespace utility
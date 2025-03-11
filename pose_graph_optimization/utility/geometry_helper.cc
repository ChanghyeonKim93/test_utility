#include "utility/geometry_helper.h"

namespace utility {

const Eigen::Matrix3d GeometryHelper::GenerateSkewSymmetricMatrix(
    const Eigen::Vector3d& vec) {
  Eigen::Matrix3d S{Eigen::Matrix3d::Zero()};
  // clang-format off
  S(0,1) = -vec(2); S(0,2) =  vec(1); S(1,2) = -vec(0);
  S(1,0) =  vec(2); S(2,0) = -vec(1); S(2,1) =  vec(0);
  // clang-format on
  return S;
}

const Eigen::Quaterniond GeometryHelper::ConvertToQuaternion(
    const Eigen::Vector3d& rvec) {
  Eigen::Quaterniond quaternion{Eigen::Quaterniond::Identity()};
  const double theta = rvec.norm();
  if (theta > std::numeric_limits<double>::epsilon()) {
    const double half_theta = theta * 0.5;
    const double sin_alpha_div_theta = std::sin(half_theta) / theta;
    quaternion.w() = std::cos(half_theta);
    quaternion.x() = rvec.x() * sin_alpha_div_theta;
    quaternion.y() = rvec.y() * sin_alpha_div_theta;
    quaternion.z() = rvec.z() * sin_alpha_div_theta;
  } else {
    quaternion.w() = 1.0;
    quaternion.x() = 0.0;
    quaternion.y() = 0.0;
    quaternion.z() = 0.0;
  }
  return quaternion;
}

const Eigen::Quaterniond GeometryHelper::ConvertToQuaternion(
    const Eigen::Matrix3d& rotation_matrix) {
  const Eigen::Matrix3d& R = rotation_matrix;
  Eigen::Quaterniond quaternion{Eigen::Quaterniond::Identity()};

  const double trace = R(0, 0) + R(1, 1) + R(2, 2);
  if (trace > 0.0) {  // I changed M_EPSILON to 0
    const double s = 0.5 / std::sqrt(trace + 1.0);
    quaternion.w() = 0.25 / s;
    quaternion.x() = (R(2, 1) - R(1, 2)) * s;
    quaternion.y() = (R(0, 2) - R(2, 0)) * s;
    quaternion.z() = (R(1, 0) - R(0, 1)) * s;
  } else {
    if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
      const double s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
      quaternion.w() = (R(2, 1) - R(1, 2)) / s;
      quaternion.x() = 0.25 * s;
      quaternion.y() = (R(0, 1) + R(1, 0)) / s;
      quaternion.z() = (R(0, 2) + R(2, 0)) / s;
    } else if (R(1, 1) > R(2, 2)) {
      const double s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
      quaternion.w() = (R(0, 2) - R(2, 0)) / s;
      quaternion.x() = (R(0, 1) + R(1, 0)) / s;
      quaternion.y() = 0.25 * s;
      quaternion.z() = (R(1, 2) + R(2, 1)) / s;
    } else {
      const double s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
      quaternion.w() = (R(1, 0) - R(0, 1)) / s;
      quaternion.x() = (R(0, 2) + R(2, 0)) / s;
      quaternion.y() = (R(1, 2) + R(2, 1)) / s;
      quaternion.z() = 0.25 * s;
    }
  }

  // const double trace = R(0, 0) + R(1, 1) + R(2, 2);
  // if (trace >= 0.0) {
  //   double t = std::sqrt(trace + 1.0);
  //   quaternion.w() = 0.5 * t;
  //   t = 0.5 / t;
  //   quaternion.x() = (R(2, 1) - R(1, 2)) * t;
  //   quaternion.y() = (R(0, 2) - R(2, 0)) * t;
  //   quaternion.z() = (R(1, 0) - R(0, 1)) * t;
  // } else {
  //   static double data[4] = {0.0};
  //   int i = 0;
  //   if (R(1, 1) > R(0, 0)) i = 1;
  //   if (R(2, 2) > R(i, i)) i = 2;
  //   const int j = (i + 1) % 3;
  //   const int k = (j + 1) % 3;
  //   double t = sqrt(R(i, i) - R(j, j) - R(k, k) + 1.0);
  //   data[i + 1] = 0.5 * t;
  //   t = 0.5 / t;
  //   data[0] = (R(k, j) - R(j, k)) * t;
  //   data[j + 1] = (R(j, i) + R(i, j)) * t;
  //   data[k + 1] = (R(k, i) + R(i, k)) * t;
  //   quaternion.w() = data[0];
  //   quaternion.x() = data[1];
  //   quaternion.y() = data[2];
  //   quaternion.z() = data[3];
  // }
  return quaternion;
}

const Eigen::Vector3d GeometryHelper::ConvertToRotationVector(
    const Eigen::Matrix3d& rotation_matrix) {
  return ConvertToRotationVector(ConvertToQuaternion(rotation_matrix));
}

const Eigen::Vector3d GeometryHelper::ConvertToRotationVector(
    const Eigen::Quaterniond& quaternion) {
  Eigen::Vector3d rvec{Eigen::Vector3d::Zero()};
  double qw = quaternion.w();
  double qx = quaternion.x();
  double qy = quaternion.y();
  double qz = quaternion.z();
  const double sq_sin_half_theta = qx * qx + qy * qy + qz * qz;

  if (sq_sin_half_theta > std::numeric_limits<double>::epsilon()) {
    const double sin_half_theta = std::sqrt(sq_sin_half_theta);
    const double cos_half_theta = qw;

    const double theta =
        2.0 * ((cos_half_theta < 0.0)
                   ? std::atan2(-sin_half_theta, -cos_half_theta)
                   : std::atan2(sin_half_theta, cos_half_theta));
    const double k = theta / sin_half_theta;
    rvec.x() = qx * k;
    rvec.y() = qy * k;
    rvec.z() = qz * k;
  } else {
    rvec.x() = qx * 0.5;
    rvec.y() = qy * 0.5;
    rvec.z() = qz * 0.5;
  }
  return rvec;
}

const Eigen::Matrix3d GeometryHelper::ConvertToRotationMatrix(
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

const Eigen::Matrix3d GeometryHelper::ConvertToRotationMatrix(
    const Eigen::Vector3d& rvec) {
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  const double theta = rvec.norm();
  if (theta > std::numeric_limits<double>::epsilon()) {
    const double sin_th = std::sin(theta);
    const double cos_th = std::cos(theta);
    const double inv_theta = 1.0 / theta;
    const Eigen::Matrix3d K = GenerateSkewSymmetricMatrix(rvec) * inv_theta;
    R = Eigen::Matrix3d::Identity() + sin_th * K + (1 - cos_th) * K * K;
  }

  return R;
}

const Eigen::Matrix3d GeometryHelper::ConvertToRotationMatrix(
    const Eigen::Quaterniond& quaternion) {
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  const double a = quaternion.w();
  const double b = quaternion.x();
  const double c = quaternion.y();
  const double d = quaternion.z();

  const double aa = a * a;
  const double ab = a * b;
  const double ac = a * c;
  const double ad = a * d;
  const double bb = b * b;
  const double bc = b * c;
  const double bd = b * d;
  const double cc = c * c;
  const double cd = c * d;
  const double dd = d * d;

  const double normalizer = 1.0 / (aa + bb + cc + dd);

  R(0, 0) = (aa + bb - cc - dd) * normalizer;
  R(0, 1) = (2.0 * (bc - ad)) * normalizer;
  R(0, 2) = (2.0 * (ac + bd)) * normalizer;
  R(1, 0) = (2.0 * (ad + bc)) * normalizer;
  R(1, 1) = (aa - bb + cc - dd) * normalizer;
  R(1, 2) = (2.0 * (cd - ab)) * normalizer;
  R(2, 0) = (2.0 * (bd - ac)) * normalizer;
  R(2, 1) = (2.0 * (ab + cd)) * normalizer;
  R(2, 2) = (aa - bb - cc + dd) * normalizer;

  return R;
}

}  // namespace utility
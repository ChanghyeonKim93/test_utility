#ifndef SO3_H_
#define SO3_H_

#include "Eigen/Dense"

class SO3 {
  // Constructors
 public:
  SO3() : data_{Eigen::Quaterniond::Identity()} {}
  SO3(const SO3& input) : data_{input.data_.normalized()} {}
  SO3(const Eigen::Vector3d& rotation_vector)
      : data_{convertToQuaternion(rotation_vector).normalized()} {}
  SO3(const Eigen::Matrix3d& rotation_matrix)
      : data_{convertToQuaternion(convertToRotationVector(rotation_matrix))
                  .normalized()} {}
  SO3(const Eigen::Quaterniond& quaternion) : data_{quaternion.normalized()} {}

  // Operator overloading
 public:
  SO3& operator=(const SO3& rhs) {
    data_ = rhs.data_.normalized();
    return *this;
  }
  SO3 operator*(const SO3& rhs) {
    return SO3((data_ * rhs.data_).normalized());
  }
  Eigen::Vector3d operator*(const Eigen::Vector3d& rhs) {
    return data_.toRotationMatrix() * rhs;
  }

  SO3& operator*=(const SO3& rhs) {
    data_ = (data_ * rhs.data_).normalized();
    return *this;
  }

  SO3 inverse() const { return SO3{data_.inverse()}; }

  // Eigen::Matrix3d computeRightJacobian() const;
  // Eigen::Matrix3d computeLeftJacobian() const;

  const Eigen::Quaterniond& toQuaternion() const { return data_; }
  Eigen::Matrix3d toRotationMatrix() const { return data_.toRotationMatrix(); }
  Eigen::Vector3d toRotationVector() const {
    Eigen::Vector3d rotation_vector{Eigen::Vector3d::Zero()};
    const double sin_half_angle = data_.vec().norm();
    constexpr double kSmallNumber{1e-7};
    if (std::abs(sin_half_angle) < kSmallNumber) return rotation_vector;
    rotation_vector =
        data_.vec() * (2.0 * std::asin(sin_half_angle) / sin_half_angle);
    return rotation_vector;
  }

 private:
  Eigen::Quaterniond convertToQuaternion(
      const Eigen::Vector3d& rotation_vector) {
    Eigen::Quaterniond quaternion{Eigen::Quaterniond::Identity()};
    const double angle = rotation_vector.norm();
    if (angle < 1e-7) {
      quaternion.w() = 1.0;
      quaternion.vec() = rotation_vector;
    } else {
      const double half_angle = angle * 0.5;
      const double cos_half_angle = std::cos(half_angle);
      const double sin_half_angle_mult_inv_angle = std::sin(half_angle) / angle;
      quaternion.w() = cos_half_angle;
      quaternion.x() = sin_half_angle_mult_inv_angle * rotation_vector.x();
      quaternion.y() = sin_half_angle_mult_inv_angle * rotation_vector.y();
      quaternion.z() = sin_half_angle_mult_inv_angle * rotation_vector.z();
    }
    quaternion.normalize();
    return quaternion;
  }
  Eigen::Vector3d convertToRotationVector(
      const Eigen::Matrix3d& rotation_matrix) {
    // Logarithm map of SO(3)
    Eigen::Vector3d rotation_vector{Eigen::Vector3d::Zero()};
    const double trace =
        rotation_matrix(0, 0) + rotation_matrix(1, 1) + rotation_matrix(2, 2);
    const double cos_angle = std::clamp((trace - 1.0), -1.0, 1.0) * 0.5;
    constexpr double kSmallNumber{1e-7};
    if (1.0 - cos_angle < kSmallNumber) return Eigen::Vector3d::Zero();
    const double angle = std::acos(cos_angle);
    rotation_vector(0) = rotation_matrix(2, 1) - rotation_matrix(1, 2);
    rotation_vector(1) = rotation_matrix(0, 2) - rotation_matrix(2, 0);
    rotation_vector(2) = rotation_matrix(1, 0) - rotation_matrix(0, 1);
    rotation_vector *= (angle * 0.5 / std::sin(angle));
    return rotation_vector;
  }

  Eigen::Quaterniond data_;
};

#endif  // SO3_H_
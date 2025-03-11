#ifndef SO3_H_
#define SO3_H_

#include <type_traits>

#include "Eigen/Dense"

template <typename Scalar>
class SO3 {
  using Mat3x3 = Eigen::Matrix<Scalar, 3, 3>;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  // Constructors
 public:
  SO3() : data_{Quaternion::Identity()} {}
  SO3(const SO3& input) : data_{input.data_.normalized()} {}
  SO3(const Vec3& rotation_vector)
      : data_{convertToQuaternion(rotation_vector).normalized()} {}
  SO3(const Mat3x3& rotation_matrix)
      : data_{convertToQuaternion(convertToRotationVector(rotation_matrix))
                  .normalized()} {}
  SO3(const Quaternion& quaternion) : data_{quaternion.normalized()} {}

  // Operator overloading
 public:
  SO3& operator=(const SO3& rhs) {
    data_ = rhs.data_.normalized();
    return *this;
  }
  SO3 operator*(const SO3& rhs) {
    return SO3((data_ * rhs.data_).normalized());
  }
  Vec3 operator*(const Vec3& rhs) { return data_.toRotationMatrix() * rhs; }

  SO3& operator*=(const SO3& rhs) {
    data_ = (data_ * rhs.data_).normalized();
    return *this;
  }

  SO3 inverse() const { return SO3{data_.inverse()}; }

  // Eigen::Matrix3d computeRightJacobian() const;
  // Eigen::Matrix3d computeLeftJacobian() const;

  const Quaternion& toQuaternion() const { return data_; }
  Mat3x3 toRotationMatrix() const { return data_.toRotationMatrix(); }
  Vec3 toRotationVector() const {
    Vec3 rotation_vector{Vec3::Zero()};
    const double sin_half_angle = data_.vec().norm();
    constexpr double kSmallNumber{1e-7};
    if (std::abs(sin_half_angle) < kSmallNumber) return rotation_vector;
    rotation_vector =
        data_.vec() * (2.0 * std::asin(sin_half_angle) / sin_half_angle);
    return rotation_vector;
  }

 private:
  Quaternion convertToQuaternion(const Vec3& rotation_vector) {
    Quaternion quaternion{Quaternion::Identity()};
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
  Vec3 convertToRotationVector(const Mat3x3& rotation_matrix) {
    // Logarithm map of SO(3)
    Vec3 rotation_vector{Vec3::Zero()};
    const double trace =
        rotation_matrix(0, 0) + rotation_matrix(1, 1) + rotation_matrix(2, 2);
    const double cos_angle = std::clamp((trace - 1.0), -1.0, 1.0) * 0.5;
    constexpr double kSmallNumber{1e-7};
    if (1.0 - cos_angle < kSmallNumber) return Vec3::Zero();
    const double angle = std::acos(cos_angle);
    rotation_vector(0) = rotation_matrix(2, 1) - rotation_matrix(1, 2);
    rotation_vector(1) = rotation_matrix(0, 2) - rotation_matrix(2, 0);
    rotation_vector(2) = rotation_matrix(1, 0) - rotation_matrix(0, 1);
    rotation_vector *= (angle * 0.5 / std::sin(angle));
    return rotation_vector;
  }

  Quaternion data_;
};

using SO3f = SO3<float>;
using SO3d = SO3<double>;

#endif  // SO3_H_
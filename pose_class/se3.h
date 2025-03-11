#ifndef SE3_H_
#define SE3_H_

#include "Eigen/Dense"

#include "so3.h"

template <typename Scalar>
class SE3 {
  using Mat3x3 = Eigen::Matrix<Scalar, 3, 3>;
  using Mat4x4 = Eigen::Matrix<Scalar, 4, 4>;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  // Constructors
 public:
  SE3() : rotation_data_{}, translation_data_{Vec3::Zero()} {}
  SE3(const SE3& input)
      : rotation_data_(input.rotation_data_),
        translation_data_(input.translation_data_) {}
  SE3(const SO3<Scalar>& rotation, const Vec3& translation)
      : rotation_data_(rotation), translation_data_(translation) {}
  // SE3(const Eigen::Matrix<double, 6, 1>& lie_algebra)
  //     : rotation_data_(rotation), translation_data_(translation) {}

  // Operator overloading
 public:
  SE3& operator=(const SE3& rhs) {
    rotation_data_ = rhs.rotation_data_;
    translation_data_ = rhs.translation_data_;
    return *this;
  }
  SE3 operator*(const SE3& rhs) {
    return SE3(rotation_data_ * rhs.rotation_data_,
               rotation_data_ * rhs.translation_data_ + translation_data_);
  }
  SE3& operator*=(const SE3& rhs) {
    rotation_data_ *= rhs.rotation_data_;
    translation_data_ += rotation_data_ * rhs.translation_data_;
    return *this;
  }

  Vec3 operator*(const Vec3& rhs) {
    return (rotation_data_ * rhs + translation_data_);
  }

  SE3 inverse() const {
    SE3 inverse;
    inverse.rotation_data_ = rotation_data_.inverse();
    inverse.translation_data_ = -(inverse.rotation_data_ * translation_data_);
    return inverse;
  }

  const SO3<Scalar>& rotation() const { return rotation_data_; }

  const Vec3& translation() const { return translation_data_; }

  // Eigen::Matrix<double, 6, 1> toLieAlgebra() const {
  //   Eigen::Matrix<double, 6, 1> lie_algebra{
  //       Eigen::Matrix<double, 6, 1>::Zero()};
  //   return lie_algebra;
  // };

 private:
  SO3<Scalar> rotation_data_;
  Vec3 translation_data_;
};

using SE3f = SE3<float>;
using SE3d = SE3<double>;

#endif  // SE3_H_
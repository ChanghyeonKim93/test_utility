#ifndef SE3_H_
#define SE3_H_

#include "Eigen/Dense"

#include "so3.h"

class SE3 {
  // Constructors
 public:
  SE3() : rotation_data_{}, translation_data_{Eigen::Vector3d::Zero()} {}
  SE3(const SE3& input)
      : rotation_data_(input.rotation_data_),
        translation_data_(input.translation_data_) {}
  SE3(const SO3& rotation, const Eigen::Vector3d& translation)
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

  Eigen::Vector3d operator*(const Eigen::Vector3d& rhs) {
    return (rotation_data_ * rhs + translation_data_);
  }

  SE3 inverse() const {
    SE3 inverse;
    inverse.rotation_data_ = rotation_data_.inverse();
    inverse.translation_data_ = -(inverse.rotation_data_ * translation_data_);
    return inverse;
  }

  const SO3& rotation() const { return rotation_data_; }

  const Eigen::Vector3d& translation() const { return translation_data_; }

  // Eigen::Matrix<double, 6, 1> toLieAlgebra() const {
  //   Eigen::Matrix<double, 6, 1> lie_algebra{
  //       Eigen::Matrix<double, 6, 1>::Zero()};
  //   return lie_algebra;
  // };

 private:
  SO3 rotation_data_;
  Eigen::Vector3d translation_data_;
};

#endif  // SE3_H_
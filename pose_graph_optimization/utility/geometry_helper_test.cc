#include "utility/geometry_helper.h"

#include "Eigen/Dense"
#include "gtest/gtest.h"

/// https://www.andre-gaschler.com/rotationconverter/

namespace utility {

TEST(GeometryHelperTest, ZeroAngleAxisToQuaternion) {
  static constexpr double kTolerance = 1e-7;
  Eigen::Vector3d rvec{Eigen::Vector3d::Zero()};
  double expected_q[4] = {1, 0, 0, 0};

  const auto q = GeometryHelper::ConvertToQuaternion(rvec);
  EXPECT_NEAR(q.w(), expected_q[0], kTolerance);
  EXPECT_NEAR(q.x(), expected_q[1], kTolerance);
  EXPECT_NEAR(q.y(), expected_q[2], kTolerance);
  EXPECT_NEAR(q.z(), expected_q[3], kTolerance);
}

TEST(GeometryHelperTest, RotationMatrixTest) {
  static constexpr double kTolerance = 1e-7;
  Eigen::Quaterniond q{1.0, 2.0, 7.0, 4.0};
  q.normalize();
  const Eigen::Matrix3d R = GeometryHelper::ConvertToRotationMatrix(q);
  const auto q2 = GeometryHelper::ConvertToQuaternion(R);
  const auto R2 = GeometryHelper::ConvertToRotationMatrix(q2);

  for (int j = 0; j < 3; ++j)
    for (int i = 0; i < 3; ++i) EXPECT_NEAR(R(i, j), R2(i, j), kTolerance);

  const auto rvec2 = GeometryHelper::ConvertToRotationVector(R);
  const auto R3 = GeometryHelper::ConvertToRotationMatrix(rvec2);

  for (int j = 0; j < 3; ++j)
    for (int i = 0; i < 3; ++i) EXPECT_NEAR(R(i, j), R3(i, j), kTolerance);
}

TEST(GeometryHelperTest, QuaternionTest) {
  static constexpr double kTolerance = 1e-7;
  Eigen::Quaterniond q{1.0, 2.0, 7.0, 4.0};
  q.normalize();

  const auto rvec = GeometryHelper::ConvertToRotationVector(q);
  const auto q2 = GeometryHelper::ConvertToQuaternion(rvec);
  EXPECT_NEAR(q2.w(), q.w(), kTolerance);
  EXPECT_NEAR(q2.x(), q.x(), kTolerance);
  EXPECT_NEAR(q2.y(), q.y(), kTolerance);
  EXPECT_NEAR(q2.z(), q.z(), kTolerance);
}

TEST(GeometryHelperTest, RotationVectorTest) {
  static constexpr double kTolerance = 1e-7;
  Eigen::Vector3d true_rvec;
  true_rvec << 1.1, 5.5, 5.3;

  const auto q = GeometryHelper::ConvertToQuaternion(true_rvec);
  const auto rvec = GeometryHelper::ConvertToRotationVector(q);

  auto q2 = GeometryHelper::ConvertToQuaternion(rvec);

  if (q2.w() * q.w() < 0.0) {
    q2.w() *= -1.0;
    q2.x() *= -1.0;
    q2.y() *= -1.0;
    q2.z() *= -1.0;
  }

  EXPECT_NEAR(q2.w(), q.w(), kTolerance);
  EXPECT_NEAR(q2.x(), q.x(), kTolerance);
  EXPECT_NEAR(q2.y(), q.y(), kTolerance);
  EXPECT_NEAR(q2.z(), q.z(), kTolerance);
}

}  // namespace utility
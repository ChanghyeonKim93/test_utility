#ifndef IMU_PREINTEGRATOR_H_
#define IMU_PREINTEGRATOR_H_

#include <deque>
#include <memory>
#include <vector>

#include "Eigen/Dense"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

// references: (folder navigation)
// PreintegrationBase.cpp
// ManifoldPreintegration.cpp
// PreintegrationParams.h
// ImuBias.h

#define GRAVITY_ACCELERATION 9.81

namespace imu_preintegrator {

struct Parameters {
  struct {
    struct {
      double linear_acc{1e-5};
      double angular_vel{1e-5};
    } measurement;
    struct {
      double linear_acc{1e-5};
      double angular_vel{1e-5};
    } bias;
  } noise;
};

using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec9 = Eigen::Matrix<double, 9, 1>;

using Mat33 = Eigen::Matrix<double, 3, 3>;
using Mat63 = Eigen::Matrix<double, 6, 3>;
using Mat36 = Eigen::Matrix<double, 3, 6>;
using Mat66 = Eigen::Matrix<double, 6, 6>;
using Mat99 = Eigen::Matrix<double, 9, 9>;

struct ImuFactor {
  double t_start{0.0};
  double t_end{0.0};
  double tij{0.0};

  Mat33 delRij{Mat33::Identity()};
  Vec3 delpij{Vec3::Zero()};
  Vec3 delvij{Vec3::Zero()};
  struct Jacobians {
    Mat33 delpij_dba;
    Mat33 delpij_dbg;
    Mat33 delvij_dba;
    Mat33 delvij_dbg;
    Mat33 delRij_dbg;
  } jacobians;
  Mat99 error_covariance{Mat99::Zero()};
};

struct Residuals {
  Vec3 LogR{Vec3::Zero()};
  Vec3 p{Vec3::Zero()};
  Vec3 v{Vec3::Zero()};
  struct Jacobians {
    struct {
      Mat33 dpi{Mat33::Zero()};
      Mat33 dvi{Mat33::Zero()};
      Mat33 dthi{Mat33::Zero()};
      Mat33 dpj{Mat33::Zero()};
      Mat33 dvj{Mat33::Zero()};
      Mat33 dthj{Mat33::Zero()};
      Mat33 ddba{Mat33::Zero()};
      Mat33 ddbg{Mat33::Zero()};
    } LogR;
    struct {
      Mat33 dpi{Mat33::Zero()};
      Mat33 dvi{Mat33::Zero()};
      Mat33 dthi{Mat33::Zero()};
      Mat33 dpj{Mat33::Zero()};
      Mat33 dvj{Mat33::Zero()};
      Mat33 dthj{Mat33::Zero()};
      Mat33 ddba{Mat33::Zero()};
      Mat33 ddbg{Mat33::Zero()};
    } p;
    struct {
      Mat33 dpi{Mat33::Zero()};
      Mat33 dvi{Mat33::Zero()};
      Mat33 dthi{Mat33::Zero()};
      Mat33 dpj{Mat33::Zero()};
      Mat33 dvj{Mat33::Zero()};
      Mat33 dthj{Mat33::Zero()};
      Mat33 ddba{Mat33::Zero()};
      Mat33 ddbg{Mat33::Zero()};
    } v;
  } jacobians;
};

struct ImuData {
  double time{0.0};
  Vec3 linear_acc{Vec3::Zero()};
  Vec3 angular_vel{Vec3::Zero()};
};

struct ImuBias {
  Vec3 linear_acc{0.0, 0.0, 0.0};
  Vec3 angular_vel{0.0, 0.0, 0.0};
};

struct ImuState {  // == NavState
  double time{0.0};
  Mat33 R{Mat33::Identity()};
  Vec3 p{Vec3::Zero()};
  Vec3 v{Vec3::Zero()};
};

class ImuPreintegrator {
 public:
  explicit ImuPreintegrator(const Parameters& parameters,
                            const ImuBias& initial_bias);
  void Propagate(const ImuData& imu_data);
  // void biasCorrectedDelta(const Bias& bias_i, Mat96* H);
  void CorrectImuFactorByUpdatedBias(const ImuBias& updated_bias);
  Residuals ComputeResiduals(const Mat33& Ri, const Vec3& pi, const Vec3& vi,
                             const double ti, const Mat33& Rj, const Vec3& pj,
                             const Vec3& vj, const double tj, const Vec3& dba,
                             const Vec3& dbg);

  void ResetImuFactor();
  void SetInitialPoseAndVelocity(const double time, const Vec3& position,
                                 const Mat33& rotation_matrix,
                                 const Vec3& linear_velocity);

  void RepropagateImuState();

 public:
  const ImuBias& GetImuBias() const;
  const ImuFactor& GetImuFactor() const;
  const ImuState& GetImuState() const;

 public:
  static Mat33 ConvertToRotationMatrix(const Vec3& rotation_vector);
  static Vec3 ConvertToRotationVector(const Mat33& rotation_matrix);
  static Mat33 ConvertToSkewSymmetricMatrix(const Vec3& rotation_vector);
  static Vec3 DeskewSymmetricMatrix(const Mat33& skew_mat);
  static Mat33 ComputeRightJacobianOfSO3(const Vec3& rotation_vector);
  static Mat33 ComputeInverseRightJacobianOfSO3(const Vec3& rotation_vector);

 private:
  ImuState IntegrateImuState(const ImuData& imu_data);

 private:
  const Parameters parameters_;

  bool is_pose_initialize_{false};

  ImuState prev_imu_state_;

  std::deque<ImuData> imu_data_queue_;
  ImuState imu_state_;
  ImuBias imu_bias_;
  ImuFactor imu_factor_;

  const Vec3 g_vec_{0.0, 0.0, -9.81};
};

class ImuPreintegrationCostFunctor {
 public:
  ImuPreintegrationCostFunctor(
      const std::shared_ptr<ImuPreintegrator>& imu_preintegrator)
      : imu_preintegrator_(imu_preintegrator) {}

  template <typename T>
  bool operator()(const T* const p0_ptr, const T* const rvec0_ptr,
                  const T* const p1_ptr, const T* const rvec1_ptr,
                  const T* const dba_ptr, const T* const dbg_ptr,
                  T* residual_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p0(p0_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p1(p1_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> dba(dba_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> dbg(dbg_ptr);

    T q0_tmp[4];
    T q1_tmp[4];
    ceres::AngleAxisToQuaternion(rvec0_ptr, q0_tmp);
    ceres::AngleAxisToQuaternion(rvec1_ptr, q1_tmp);
    // ceres::AngleAxisToRotationMatrix(rvec0_ptr);
    // reorder w,x,y,z -> x,y,z,w
    T q0w = q0_tmp[0];
    q0_tmp[0] = q0_tmp[1];
    q0_tmp[1] = q0_tmp[2];
    q0_tmp[2] = q0_tmp[3];
    q0_tmp[3] = q0w;
    T q1w = q1_tmp[0];
    q1_tmp[0] = q1_tmp[1];
    q1_tmp[1] = q1_tmp[2];
    q1_tmp[2] = q1_tmp[3];
    q1_tmp[3] = q1w;
    Eigen::Map<const Eigen::Quaternion<T>> q0(q0_tmp);
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_tmp);

    // ceres::AngleAxisToRotationMatrix(rvec0_ptr, R0);
    // ceres::AngleAxisToRotationMatrix(rvec1_ptr, R1);

    const auto imu_factor = imu_preintegrator_->GetImuFactor();
    const auto delpij = imu_factor.delpij.template cast<T>();
    const auto delvij = imu_factor.delvij.template cast<T>();
    const auto delRij = imu_factor.delRij.template cast<T>();
    const auto delpij_dba = imu_factor.jacobians.delpij_dba.template cast<T>();
    const auto delpij_dbg = imu_factor.jacobians.delpij_dbg.template cast<T>();
    const auto delvij_dba = imu_factor.jacobians.delvij_dba.template cast<T>();
    const auto delvij_dbg = imu_factor.jacobians.delvij_dbg.template cast<T>();

    Eigen::Map<Eigen::Matrix<T, 9, 1>> residual(residual_ptr);
    residual.template block<3, 1>(
        0, 0);  // LogR = Ri.transpose()* (pj-pi-am*dt-g*dt*dt)-
                // (del_p+dp_dba*dba+dp_dbg*dbg);
    residual.template block<3, 1>(3, 0);  // p
    residual.template block<3, 1>(6, 0);  // v

    // ceres::RotationMatrixToAngleAxis();
  }

 private:
  std::shared_ptr<ImuPreintegrator> imu_preintegrator_{nullptr};
};

}  // namespace imu_preintegrator

#endif  // IMU_PREINTEGRATOR_H_
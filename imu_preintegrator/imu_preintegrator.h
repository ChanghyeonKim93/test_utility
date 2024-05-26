#ifndef IMU_PREINTEGRATOR_H_
#define IMU_PREINTEGRATOR_H_

#include <deque>
#include <vector>

#include "Eigen/Dense"

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
                             const Vec3& vj, const double tj, const Vec3& dpi,
                             const Vec3& dvi, const Vec3& dthi, const Vec3& dpj,
                             const Vec3& dvj, const Vec3& dthj, const Vec3& dba,
                             const Vec3& dbg);

  void ResetImuFactor();
  void InitializePoseAndVelocity(const double time, const Vec3& position,
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

}  // namespace imu_preintegrator

#endif  // IMU_PREINTEGRATOR_H_
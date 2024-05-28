#ifndef IMU_PREINTEGRATOR_H_
#define IMU_PREINTEGRATOR_H_

#include <deque>
#include <memory>
#include <vector>

#include "Eigen/Dense"

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

using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using Vec15 = Eigen::Matrix<double, 15, 1>;

using Mat33 = Eigen::Matrix<double, 3, 3>;
using Mat63 = Eigen::Matrix<double, 6, 3>;
using Mat36 = Eigen::Matrix<double, 3, 6>;
using Mat66 = Eigen::Matrix<double, 6, 6>;
using Mat99 = Eigen::Matrix<double, 9, 9>;
using Mat15_9 = Eigen::Matrix<double, 15, 9>;
using Mat15_15 = Eigen::Matrix<double, 15, 15>;

struct ImuPreintegration {
  double time_start{0.0};
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

struct ImuPreintegrationResidual {
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

struct ImuState {
  double time{0.0};
  Mat33 R{Mat33::Identity()};
  Vec3 p{Vec3::Zero()};
  Vec3 v{Vec3::Zero()};
};

class ImuPreintegrator {
 public:
  /// @brief Constructor
  /// @param parameters Default parameters
  /// @param initial_imu_bias Initial imu bias
  explicit ImuPreintegrator(const Parameters& parameters,
                            const ImuBias& initial_imu_bias);

  /// @brief Integrate imu preintegration and imu state. Their related Jacobians
  /// are also updated sequentially.
  /// @param imu_data Current imu data
  void Integrate(const ImuData& imu_data);

  /// @brief Correct imu preintegration measurements using updated bias by
  /// first-order update scheme. Private imu bias value is changed to the input
  /// updated bias.
  /// @param updated_bias Updated bias
  void CorrectImuPreintegrationByUpdatedBias(const ImuBias& updated_bias);

  /// @brief Compute residuals and their Jacobians for imu preintegration
  /// measurements. All input rotations, positions, and velocities are
  /// represented in the world frame.
  /// @param Ri i-th IMU rotation matrix
  /// @param pi i-th IMU position vector
  /// @param vi i-th IMU linear velocity
  /// @param ti i-th time
  /// @param Rj j-th IMU rotation matrix
  /// @param pj j-th IMU position vector
  /// @param vj j-th IMU linear velocity
  /// @param tj j-th time
  /// @param dba small update on linear acc bias
  /// @param dbg small update on angular vel bias
  /// @return ImuPreintegrationResidual struct
  ImuPreintegrationResidual ComputeImuPreintegrationResidual(
      const Mat33& Ri, const Vec3& pi, const Vec3& vi, const double ti,
      const Mat33& Rj, const Vec3& pj, const Vec3& vj, const double tj,
      const Vec3& dba, const Vec3& dbg);

  /// @brief Repropagate imu state. This function might be called after bias
  /// correction. The queued imu data is used to re-propagate imu state from the
  /// reference imu state for integration which is internally reserved in the
  /// class.
  void RepropagateImuState();

  /// @brief Reset imu preintegration. This function resets the imu
  /// preintegration measurements and their Jacobians. Imu state, linear
  /// velocity and imu bias are not affected by this function.
  void ResetImuPreintegration();

  void SetInitialPoseAndVelocity(const double time, const Mat33& R,
                                 const Vec3& p, const Vec3& v);

  /// @brief Get current imu bias
  /// @return Current imu bias
  const ImuBias& GetImuBias() const;

  /// @brief Get current imu state
  /// @return Current imu state
  const ImuState& GetImuState() const;

  /// @brief Get current imu preintegration struct
  /// @return Current imu preintegration struct
  const ImuPreintegration& GetImuPreintegration() const;

 public:
  static Mat33 ConvertToRotationMatrix(const Vec3& rotation_vector);
  static Vec3 ConvertToRotationVector(const Mat33& rotation_matrix);
  static Mat33 ConvertToSkewSymmetricMatrix(const Vec3& rotation_vector);
  static Vec3 DeskewSymmetricMatrix(const Mat33& skew_mat);
  static Mat33 ComputeRightJacobianOfSO3(const Vec3& rotation_vector);
  static Mat33 ComputeInverseRightJacobianOfSO3(const Vec3& rotation_vector);

 private:
  ImuState IntegrateImuState(const ImuState& imu_state,
                             const ImuData& imu_data);

 private:
  const Parameters parameters_;

  bool is_pose_initialize_{false};

  ImuState reference_imu_state_for_integration_;
  std::deque<ImuData> imu_data_queue_;

  ImuState imu_state_;
  ImuBias imu_bias_;
  ImuPreintegration imu_factor_;

  Mat66 imu_noise_covariance_;

  const Vec3 gravity_vector_{0.0, 0.0, -GRAVITATIONAL_ACCELERATION};
};

}  // namespace imu_preintegrator

#endif  // IMU_PREINTEGRATOR_H_
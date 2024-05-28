#include "imu_preintegrator.h"

#include <iostream>
#include <limits>

namespace imu_preintegrator {

ImuPreintegrator::ImuPreintegrator(const Parameters& parameters,
                                   const ImuBias& initial_bias)
    : parameters_(parameters), imu_bias_(initial_bias) {
  ResetImuPreintegration();
  imu_noise_covariance_.setZero();
  const double noise_acc = parameters.noise.measurement.linear_acc;
  const double noise_gyro = parameters.noise.measurement.angular_vel;
  imu_noise_covariance_.block<3, 3>(0, 0) =
      Mat33::Identity() * noise_acc * noise_acc;
  imu_noise_covariance_.block<3, 3>(3, 3) =
      Mat33::Identity() * noise_gyro * noise_gyro;
}

void ImuPreintegrator::ResetImuPreintegration() {
  imu_data_queue_.clear();

  imu_factor_.delRij.setIdentity();
  imu_factor_.delpij.setZero();
  imu_factor_.delvij.setZero();
  imu_factor_.jacobians.delRij_dbg.setZero();
  imu_factor_.jacobians.delpij_dba.setZero();
  imu_factor_.jacobians.delpij_dbg.setZero();
  imu_factor_.jacobians.delvij_dba.setZero();
  imu_factor_.jacobians.delvij_dbg.setZero();
  imu_factor_.tij = 0.0;
  imu_factor_.time_start = 0.0;
  imu_factor_.error_covariance.setZero();
}

void ImuPreintegrator::SetInitialPoseAndVelocity(const double time,
                                                 const Mat33& R, const Vec3& p,
                                                 const Vec3& v) {
  if (!is_pose_initialize_) {
    imu_state_.R = R;
    imu_state_.p = p;
    imu_state_.v = v;
    is_pose_initialize_ = true;
  }
}

void ImuPreintegrator::RepropagateImuState() {
  ImuState temporary_imu_state = reference_imu_state_for_integration_;
  for (const auto& imu_data : imu_data_queue_)
    temporary_imu_state = IntegrateImuState(temporary_imu_state, imu_data);
  imu_state_ = temporary_imu_state;
}

void ImuPreintegrator::Integrate(const ImuData& imu_data) {
  if (imu_data_queue_.empty()) {
    reference_imu_state_for_integration_ = imu_state_;
    imu_factor_.time_start = imu_data.time;
    imu_data_queue_.push_back(imu_data);
    return;
  }

  if (imu_data_queue_.back().time > imu_data.time) return;  // skip
  const double dt = imu_data.time - imu_state_.time;
  if (dt < 0.001) return;

  imu_data_queue_.push_back(imu_data);

  imu_state_ = IntegrateImuState(imu_state_, imu_data);

  // Compensate bias on imu measurements
  const Vec3 compensated_linear_acc =
      imu_data.linear_acc - imu_bias_.linear_acc;
  const Vec3 compensated_angular_vel =
      imu_data.angular_vel - imu_bias_.angular_vel;

  // Update imu preintegration measurements
  imu_factor_.tij += dt;
  auto& delRij = imu_factor_.delRij;
  auto& delpij = imu_factor_.delpij;
  auto& delvij = imu_factor_.delvij;
  const Mat33 old_delRij = delRij;
  const Mat33 R_increment =
      ConvertToRotationMatrix(compensated_angular_vel * dt);

  delRij *= R_increment;
  delpij += delvij * dt + 0.5 * old_delRij * compensated_linear_acc * dt * dt;
  delvij += old_delRij * compensated_linear_acc * dt;

  // Update Jacobians
  auto& delRij_dbg = imu_factor_.jacobians.delRij_dbg;
  auto& delpij_dba = imu_factor_.jacobians.delpij_dba;
  auto& delpij_dbg = imu_factor_.jacobians.delpij_dbg;
  auto& delvij_dba = imu_factor_.jacobians.delvij_dba;
  auto& delvij_dbg = imu_factor_.jacobians.delvij_dbg;
  const auto old_delRij_dbg = delRij_dbg;
  const auto old_delvij_dba = delvij_dba;
  const auto old_delvij_dbg = delvij_dbg;
  const Mat33 am_ba_skew = ConvertToSkewSymmetricMatrix(compensated_linear_acc);
  delRij_dbg = R_increment.transpose() * delRij_dbg -
               ComputeRightJacobianOfSO3(compensated_angular_vel * dt) * dt;
  delpij_dba += (old_delvij_dba * dt - 0.5 * old_delRij * dt * dt);
  delpij_dbg += (old_delvij_dbg * dt -
                 0.5 * old_delRij * am_ba_skew * old_delRij_dbg * dt * dt);
  delvij_dba += -(old_delRij * dt);
  delvij_dbg += -(old_delRij * am_ba_skew * old_delRij_dbg * dt);

  // Update covariance
  imu_factor_.error_covariance;
}

void ImuPreintegrator::CorrectImuPreintegrationByUpdatedBias(
    const ImuBias& updated_bias) {
  const Vec3 dba = updated_bias.linear_acc - imu_bias_.linear_acc;
  const Vec3 dbg = updated_bias.angular_vel - imu_bias_.angular_vel;

  auto& delRij = imu_factor_.delRij;
  auto& delpij = imu_factor_.delpij;
  auto& delvij = imu_factor_.delvij;
  const auto& delRij_dbg = imu_factor_.jacobians.delRij_dbg;
  const auto& delpij_dba = imu_factor_.jacobians.delpij_dba;
  const auto& delpij_dbg = imu_factor_.jacobians.delpij_dbg;
  const auto& delvij_dba = imu_factor_.jacobians.delvij_dba;
  const auto& delvij_dbg = imu_factor_.jacobians.delvij_dbg;

  delRij.noalias() = delRij * ConvertToRotationMatrix(delRij_dbg * dbg);
  delpij.noalias() = delpij + delpij_dba * dba + delpij_dbg * dbg;
  delvij.noalias() = delvij + delvij_dba * dba + delvij_dbg * dbg;

  imu_bias_ = updated_bias;
}

ImuPreintegrationResidual ImuPreintegrator::ComputeResiduals(
    const Mat33& Ri, const Vec3& pi, const Vec3& vi, const double ti,
    const Mat33& Rj, const Vec3& pj, const Vec3& vj, const double tj,
    const Vec3& dpi, const Vec3& dvi, const Vec3& dthi, const Vec3& dpj,
    const Vec3& dvj, const Vec3& dthj, const Vec3& dba, const Vec3& dbg) {
  ImuPreintegrationResidual residuals;

  const double tij = tj - ti;

  const auto& delRij = imu_factor_.delRij;
  const auto& delpij = imu_factor_.delpij;
  const auto& delvij = imu_factor_.delvij;

  const auto& delRij_dbg = imu_factor_.jacobians.delRij_dbg;
  const auto& delpij_dba = imu_factor_.jacobians.delpij_dba;
  const auto& delpij_dbg = imu_factor_.jacobians.delpij_dbg;
  const auto& delvij_dba = imu_factor_.jacobians.delvij_dba;
  const auto& delvij_dbg = imu_factor_.jacobians.delvij_dbg;

  residuals.LogR = ConvertToRotationVector(
      ((delRij * ConvertToRotationMatrix(delRij_dbg * dbg)).transpose() *
       Ri.transpose() * Rj));
  residuals.p =
      Ri.transpose() * (pj - pi - vi * tij - 0.5 * gravity_vec_ * tij * tij) -
      (delpij + delpij_dba * dba + delpij_dbg * dbg);
  residuals.v = Ri.transpose() * (vj - vi - gravity_vec_ * tij) -
                (delvij + delvij_dba * dba + delvij_dbg * dbg);

  return residuals;
}

const ImuBias& ImuPreintegrator::GetImuBias() const { return imu_bias_; }

const ImuPreintegration& ImuPreintegrator::GetImuPreintegration() const {
  return imu_factor_;
}

const ImuState& ImuPreintegrator::GetImuState() const { return imu_state_; }

ImuState ImuPreintegrator::IntegrateImuState(const ImuState& imu_state,
                                             const ImuData& imu_data) {
  if (!is_pose_initialize_) {
    std::cerr << "It is not initialized.\n";
    return imu_state;
  }

  const double dt = imu_data.time - imu_state.time;
  if (dt < 0.001) return imu_state;

  const Vec3 compensated_linear_acc =
      imu_data.linear_acc - imu_bias_.linear_acc;
  const Vec3 compensated_angular_vel =
      imu_data.angular_vel - imu_bias_.angular_vel;

  ImuState new_imu_state;
  new_imu_state.time = imu_data.time;
  new_imu_state.R =
      imu_state.R * ConvertToRotationMatrix(compensated_angular_vel * dt);
  new_imu_state.v = imu_state.v + gravity_vector_ * dt +
                    new_imu_state.R * compensated_linear_acc * dt;
  new_imu_state.p = imu_state.p + imu_state.v * dt +
                    0.5 * gravity_vector_ * dt * dt +
                    0.5 * new_imu_state.R * compensated_linear_acc * dt * dt;
  return new_imu_state;
}

Mat33 ImuPreintegrator::ConvertToRotationMatrix(const Vec3& rotation_vector) {
  Mat33 R{Mat33::Identity()};
  const double angle = rotation_vector.norm();
  if (angle > std::numeric_limits<double>::min()) {
    const double squared_angle = angle * angle;
    const double inverse_angle = 1.0 / angle;
    const Mat33 rvec_skew = ConvertToSkewSymmetricMatrix(rotation_vector);
    R = Mat33::Identity() + std::sin(angle) * inverse_angle * rvec_skew +
        (1.0 - std::cos(angle)) * (inverse_angle * inverse_angle) * rvec_skew *
            rvec_skew;
  }

  return R;
}

Vec3 ImuPreintegrator::ConvertToRotationVector(const Mat33& rotation_matrix) {
  Vec3 rotation_vector{Vec3::Zero()};
  const double trace = rotation_matrix.trace();
  const double angle = std::acos(std::clamp((trace - 1.0) * 0.5, -1.0, 1.0));
  if (std::abs(angle) < std::numeric_limits<double>::min())
    return Vec3{0.0, 0.0, angle};

  Mat33 skew_mat = angle * (rotation_matrix - rotation_matrix.transpose()) /
                   (2.0 * std::sin(angle));
  rotation_vector = DeskewSymmetricMatrix(skew_mat);

  return rotation_vector;
}

Mat33 ImuPreintegrator::ConvertToSkewSymmetricMatrix(
    const Vec3& rotation_vector) {
  Mat33 S{Mat33::Zero()};
  S(0, 1) = -rotation_vector.z();
  S(0, 2) = rotation_vector.y();
  S(1, 2) = -rotation_vector.x();
  S(1, 0) = rotation_vector.z();
  S(2, 0) = -rotation_vector.y();
  S(2, 1) = rotation_vector.x();
  return S;
}

Vec3 ImuPreintegrator::DeskewSymmetricMatrix(const Mat33& skew_mat) {
  Vec3 vec;
  vec(0) = -skew_mat(1, 2);
  vec(1) = skew_mat(0, 2);
  vec(2) = -skew_mat(0, 1);
  return vec;
}

Mat33 ImuPreintegrator::ComputeRightJacobianOfSO3(const Vec3& rotation_vector) {
  Mat33 right_jacobian{Mat33::Identity()};
  const double angle = rotation_vector.norm();
  if (angle > std::numeric_limits<double>::min()) {
    const Mat33 rvec_skew = ConvertToSkewSymmetricMatrix(rotation_vector);
    right_jacobian = Mat33::Identity() -
                     (1.0 - std::cos(angle)) / (angle * angle) * rvec_skew +
                     (angle - std::sin(angle)) / (angle * angle * angle) *
                         rvec_skew * rvec_skew;
  }
  return right_jacobian;
}

Mat33 ImuPreintegrator::ComputeInverseRightJacobianOfSO3(
    const Vec3& rotation_vector) {
  Mat33 inverse_right_jacobian{Mat33::Identity()};
  const double angle = rotation_vector.norm();
  if (angle > std::numeric_limits<double>::min()) {
    const Mat33 rvec_skew = ConvertToSkewSymmetricMatrix(rotation_vector);
    inverse_right_jacobian =
        Mat33::Identity() + 0.5 * rvec_skew +
        (1.0 / (angle * angle) -
         (1.0 + std::cos(angle)) / (2.0 * angle * std::sin(angle))) *
            rvec_skew * rvec_skew;
  }
  return inverse_right_jacobian;
}

}  // namespace imu_preintegrator

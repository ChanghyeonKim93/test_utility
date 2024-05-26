#include "imu_preintegrator.h"

#include <iostream>
#include <limits>

namespace imu_preintegrator {

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
        (1.0 / (angle * angle) +
         (1.0 + std::cos(angle)) / (2.0 * angle * std::sin(angle))) *
            rvec_skew * rvec_skew;
  }
  return inverse_right_jacobian;
}

ImuPreintegrator::ImuPreintegrator(const Parameters& parameters,
                                   const ImuBias& initial_bias)
    : parameters_(parameters), imu_bias_(initial_bias) {
  ResetImuFactor();
}

void ImuPreintegrator::ResetImuFactor() {
  imu_data_queue_.clear();

  imu_factor_.delpij.setZero();
  imu_factor_.delvij.setZero();
  imu_factor_.delRij.setIdentity();
  imu_factor_.error_covariance.setZero();
  // imu_factor_.jacobians.delpij_dba.setIdentity();
  // imu_factor_.jacobians.delpij_dbg.setIdentity();
  // imu_factor_.jacobians.delvij_dba.setIdentity();
  // imu_factor_.jacobians.delvij_dbg.setIdentity();
  // imu_factor_.jacobians.delRij_dbg.setIdentity();
  imu_factor_.jacobians.delpij_dba.setZero();
  imu_factor_.jacobians.delpij_dbg.setZero();
  imu_factor_.jacobians.delvij_dba.setZero();
  imu_factor_.jacobians.delvij_dbg.setZero();
  imu_factor_.jacobians.delRij_dbg.setZero();
  imu_factor_.tij = 0.0;
  imu_factor_.t_start = 0.0;
  imu_factor_.t_end = 0.0;
}

void ImuPreintegrator::InitializePoseAndVelocity(const double time,
                                                 const Vec3& position,
                                                 const Mat33& rotation_matrix,
                                                 const Vec3& linear_velocity) {
  if (!is_pose_initialize_) {
    imu_state_.p = position;
    imu_state_.R = rotation_matrix;
    imu_state_.v = linear_velocity;
    is_pose_initialize_ = true;
  }
}

void ImuPreintegrator::RepropagateImuState() {
  imu_state_ = prev_imu_state_;
  for (const auto& imu_data : imu_data_queue_)
    imu_state_ = IntegrateImuState(imu_data);
}

void ImuPreintegrator::Propagate(const ImuData& imu_data) {
  if (imu_data_queue_.empty()) {
    prev_imu_state_ = imu_state_;
    imu_factor_.t_start = imu_data.time;
    imu_data_queue_.push_back(imu_data);
    return;
  }

  if (imu_data_queue_.back().time > imu_data.time) return;  // skip

  const double current_time = imu_data.time;
  const double dt = imu_data.time - imu_data_queue_.back().time;
  if (dt < 0.001) return;

  imu_data_queue_.push_back(imu_data);
  const auto prev_imu_data = *(imu_data_queue_.end() - 2);
  imu_state_ = IntegrateImuState(prev_imu_data);

  // Compensate bias on imu measurements
  const Vec3& ba = imu_bias_.linear_acc;
  const Vec3& bg = imu_bias_.angular_vel;
  const Vec3 am = prev_imu_data.linear_acc;
  const Vec3 wm = prev_imu_data.angular_vel;

  // Update imu preintegration measurements
  imu_factor_.t_end = imu_data.time;
  imu_factor_.tij += dt;
  auto& delRij = imu_factor_.delRij;
  auto& delpij = imu_factor_.delpij;
  auto& delvij = imu_factor_.delvij;
  const Mat33 old_delRij = delRij;
  const Mat33 R_increment = ConvertToRotationMatrix((wm - bg) * dt);

  delRij *= R_increment;
  delpij += delvij * dt + 0.5 * old_delRij * (am - ba) * dt * dt;
  delvij += old_delRij * (am - ba) * dt;

  // Update Jacobians
  auto& delRij_dbg = imu_factor_.jacobians.delRij_dbg;
  auto& delpij_dba = imu_factor_.jacobians.delpij_dba;
  auto& delpij_dbg = imu_factor_.jacobians.delpij_dbg;
  auto& delvij_dba = imu_factor_.jacobians.delvij_dba;
  auto& delvij_dbg = imu_factor_.jacobians.delvij_dbg;
  const auto old_delRij_dbg = delRij_dbg;
  const auto old_delvij_dba = delvij_dba;
  const auto old_delvij_dbg = delvij_dbg;
  const Mat33 am_ba_skew = ConvertToSkewSymmetricMatrix(am - ba);
  // delRij_dbg += -(R_increment.transpose() *
  //                 ComputeRightJacobianOfSO3((wm - bg) * dt) * dt);
  delRij_dbg = R_increment.transpose() * delRij_dbg -
               ComputeRightJacobianOfSO3((wm - bg) * dt) * dt;
  delpij_dba += (old_delvij_dba * dt - 0.5 * old_delRij * dt * dt);
  delpij_dbg += (old_delvij_dbg * dt -
                 0.5 * old_delRij * am_ba_skew * old_delRij_dbg * dt * dt);
  delvij_dba += -(old_delRij * dt);
  delvij_dbg += -(old_delRij * am_ba_skew * old_delRij_dbg * dt);
}

void ImuPreintegrator::CorrectImuFactorByUpdatedBias(
    const ImuBias& updated_bias) {
  // Reference :
  // https://github1s.com/borglab/gtsam/blob/develop/gtsam/navigation/ManifoldPreintegration.cpp
  const Vec3 dba = updated_bias.linear_acc - imu_bias_.linear_acc;
  const Vec3 dbg = updated_bias.angular_vel - imu_bias_.angular_vel;

  imu_bias_ = updated_bias;

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
}

Residuals ImuPreintegrator::ComputeResiduals(
    const Mat33& Ri, const Vec3& pi, const Vec3& vi, const double ti,
    const Mat33& Rj, const Vec3& pj, const Vec3& vj, const double tj,
    const Vec3& dpi, const Vec3& dvi, const Vec3& dthi, const Vec3& dpj,
    const Vec3& dvj, const Vec3& dthj, const Vec3& dba, const Vec3& dbg) {
  Residuals residuals;

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
      Ri.transpose() * (pj - pi - vi * tij - 0.5 * g_vec_ * tij * tij) -
      (delpij + delpij_dba * dba + delpij_dbg * dbg);
  residuals.v = Ri.transpose() * (vj - vi - g_vec_ * tij) -
                (delvij + delvij_dba * dba + delvij_dbg * dbg);

  return residuals;
}

const ImuBias& ImuPreintegrator::GetImuBias() const { return imu_bias_; }

const ImuFactor& ImuPreintegrator::GetImuFactor() const { return imu_factor_; }

const ImuState& ImuPreintegrator::GetImuState() const { return imu_state_; }

ImuState ImuPreintegrator::IntegrateImuState(const ImuData& imu_data) {
  if (!is_pose_initialize_) {
    std::cerr << "It is not initialized.\n";
    return imu_state_;
  }

  const double dt = imu_data.time - imu_state_.time;
  if (dt < 0.001) return imu_state_;

  const Vec3 am = imu_data.linear_acc - imu_bias_.linear_acc;
  const Vec3 wm = imu_data.angular_vel - imu_bias_.angular_vel;

  ImuState new_imu_state;
  new_imu_state.time = imu_data.time;

  new_imu_state.R = imu_state_.R * ConvertToRotationMatrix(wm * dt);
  new_imu_state.v = imu_state_.v + g_vec_ * dt + new_imu_state.R * am * dt;
  new_imu_state.p = imu_state_.p + imu_state_.v * dt + 0.5 * g_vec_ * dt * dt +
                    0.5 * new_imu_state.R * am * dt * dt;

  return new_imu_state;
}

}  // namespace imu_preintegrator

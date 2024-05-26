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

void ImuPreintegrator::Propagate(const ImuData& imu_data) {
  if (imu_data_queue_.empty()) {
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
  std::cerr << "dba: " << dba.transpose() << ", dbg: " << dbg.transpose()
            << std::endl;
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

const ImuBias& ImuPreintegrator::GetImuBias() const { return imu_bias_; }

const ImuFactor& ImuPreintegrator::GetImuFactor() const { return imu_factor_; }

}  // namespace imu_preintegrator

/*
소액체당금 신청 가능 (법률구조공단)

1.폐업 시에도 급여 및 퇴직금 등 금품청산이 이루어져야 합니다.

2.폐업 소액체당금과 일반체당금의 신청이 가능합니다.

3.소액체당금 수령을 위하여는 일차적으로 관할 고용노동관서에 진정/고소를 제기하여
체불임금확인(사업주임금체불확인서)을 받아야 합니다. 체불임금이 확인된 경우,
민사소송을 진행하여 판결문을 수령한 후 관할 근로복지공단에 소액체당금을 신청하는
것이 가능합니다.

4.일반체당금 수령을 위하여는 체불임금확인을 받은 후 폐업사실을 증명하는 서류와
함께 관할 근로복지공단에 일반체당금 신청을 하게 됩니다.

사업주 대신 국가가,임금,퇴직금 등 일부를 대신 지급하는체당금제도가 있습니다.
체당금 제도를 활용하면 미지급 퇴직금,임금의일부를 받을 수 있습니다.

근로기준법 제36조에 따라 14일 이내 임금을 지급해야하며, 미지급시 임금체불에
해당.
 ※ 근로기준법 제36조(금품 청산) 사용자는 근로자가 사망 또는 퇴직한 경우에는
그 지급 사유가 발생한 때부터 14일 이내에 임금, 보상금, 그 밖의 모든 금품을
지급하여야 한다. 다만, 특별한 사정이 있을 경우에는 당사자 사이의 합의에 의하여
기일을 연장할 수 있다.

※ 근로기준법 제37조(미지급 임금에 대한 지연이자) ① 사용자는 제36조에 따라
지급하여야 하는 임금 및 「근로자퇴직급여 보장법」 제2조제5호에 따른
급여(일시금만 해당된다)의 전부 또는 일부를 그 지급 사유가 발생한 날부터 14일
이내에 지급하지 아니한 경우 그 다음 날부터 지급하는 날 까지의 지연 일수에 대하여
연 100분의 40 이내의 범위에서 「은행법」에 따른 은행이 적용하는 연체금리 등 경제
여건을 고려하여 대통령령으로 정하는 이율에 따른 지연이자를 지급하여야 한다.
*/
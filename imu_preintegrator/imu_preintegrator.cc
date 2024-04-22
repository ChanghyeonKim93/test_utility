#include "imu_preintegrator.h"

namespace imu_preintegrator {

ImuPreintegrator::ImuPreintegrator(
    const MeasurementCovariances& measurement_noise_cov, const Bias& bias)
    : measurement_noise_cov_(measurement_noise_cov), bias_(bias) {
  ResetIntegration();
}

void ImuPreintegrator::ResetIntegration() {
  delta_time_i_to_j_ = 0.0;
  delta_state_from_i_to_j_ = ImuState();
  dR_dbg_.setZero();
  dp_dba_.setZero();
  dp_dbg_.setZero();
  dv_dba_.setZero();
  dv_dbg_.setZero();
}

}  // namespace imu_preintegrator
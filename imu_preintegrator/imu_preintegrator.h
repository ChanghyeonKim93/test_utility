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

namespace imu_preintegrator {

#define GRAVITY_ACC 9.81

using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec9 = Eigen::Matrix<double, 9, 1>;

using Mat33 = Eigen::Matrix<double, 3, 3>;
using Mat63 = Eigen::Matrix<double, 6, 3>;
using Mat36 = Eigen::Matrix<double, 3, 6>;
using Mat66 = Eigen::Matrix<double, 6, 6>;
using Mat99 = Eigen::Matrix<double, 9, 9>;

struct MeasurementCovariances {
  Mat33 acc_noise_cov{Mat33::Identity()};
  Mat33 gyro_noise_cov{Mat33::Identity()};
  Vec3 gravity_vector{0.0, 0.0, -GRAVITY_ACC};
};

struct ImuData {
  double time{0.0};
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
};

struct Bias {
  Eigen::Vector3d ba{0.0, 0.0, 0.0};
  Eigen::Vector3d bg{0.0, 0.0, 0.0};
  Eigen::Matrix<double, 6, 1> GetVectorized() const {
    Eigen::Matrix<double, 6, 1> vectorized;
    vectorized << ba, bg;
    return vectorized;
  }
};

struct ImuState {  // == NavState
  double time{0.0};
  Mat33 R{Mat33::Identity()};
  Vec3 p{Vec3::Zero()};
  Vec3 v{Vec3::Zero()};
};

class ImuPreintegrator {
 public:
  explicit ImuPreintegrator(const MeasurementCovariances& measurement_noise_cov,
                            const Bias& bias);
  void ResetIntegration();
  void Update(const Vec3& am, const Vec3& wm,
              const double dt /*, Mat99* A, Mat93* B, Mat93* C */);
  // void biasCorrectedDelta(const Bias& bias_i, Mat96* H);

 public:
  const Bias& GetBias() const;
  const double GetTimeDifferenceBetweenItoJ() const;
  Vec3 Get_del_pij() const;
  Vec3 Get_del_vij() const;
  Mat33 Get_del_Rij() const;
  ImuState Get_del_Xij() const;

 private:
  std::deque<ImuData> imu_data_queue_;

  double delta_time_i_to_j_{0.0};
  ImuState delta_state_from_i_to_j_;
  Bias bias_;
  Mat33 dR_dbg_;
  Mat33 dp_dba_;
  Mat33 dp_dbg_;
  Mat33 dv_dba_;
  Mat33 dv_dbg_;

  const MeasurementCovariances measurement_noise_cov_;
};

}  // namespace imu_preintegrator

#endif IMU_PREINTEGRATOR_H_
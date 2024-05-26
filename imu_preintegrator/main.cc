#include <deque>
#include <iostream>
#include <memory>

#include "imu_preintegrator.h"

void TrinImuDataQueue(std::deque<imu_preintegrator::ImuData>* imu_data_queue,
                      const double trimming_start_time) {
  if (imu_data_queue->empty()) return;

  while (!imu_data_queue->empty() &&
         imu_data_queue->front().time < trimming_start_time) {
    imu_data_queue->pop_front();
  }

  static constexpr double kMaxQueueSize = 5000;
  if (imu_data_queue->size() > kMaxQueueSize) imu_data_queue->pop_front();
};

std::deque<imu_preintegrator::ImuData> FindImuDataRange(
    const std::deque<imu_preintegrator::ImuData>& raw_data_queue,
    const double time_start, const double time_end) {
  std::deque<imu_preintegrator::ImuData> imu_data_in_range;

  if (raw_data_queue.empty()) return {};
  if (raw_data_queue.front().time > time_end) return {};
  if (raw_data_queue.back().time < time_start) return {};

  return imu_data_in_range;
};

int main() {
  imu_preintegrator::Parameters parameters;
  parameters.noise.measurement.linear_acc = 1e-5;
  parameters.noise.measurement.angular_vel = 1e-5;
  parameters.noise.bias.linear_acc = 1e-7;
  parameters.noise.bias.angular_vel = 1e-7;
  imu_preintegrator::ImuBias initial_bias;
  initial_bias.linear_acc << 0.07, -0.05, 0.2;
  initial_bias.angular_vel << 0.006, 0.015, -0.05;

  // Simulate data
  std::deque<imu_preintegrator::ImuData> imu_queue_raw;
  imu_preintegrator::ImuData data;
  data.time = 0.0;
  data.linear_acc = {0.0, 0.0, 9.81};
  data.linear_acc = data.linear_acc / data.linear_acc.norm() * 9.81;
  data.angular_vel = {0.0, 0.0, 0.00};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.linear_acc = {0.03, 0.0, 9.81};
  data.linear_acc = data.linear_acc / data.linear_acc.norm() * 9.81;
  data.angular_vel = {0.0, 0.0, 0.0};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.linear_acc = {0.03, 0.0, 9.81};
  data.linear_acc = data.linear_acc / data.linear_acc.norm() * 9.81;
  data.angular_vel = {0.0, 0.0, 0.0};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.linear_acc = {0.03, 0.0, 9.81};
  data.linear_acc = data.linear_acc / data.linear_acc.norm() * 9.81;
  data.angular_vel = {0.0, 0.0, 0.0};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.0};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.linear_acc = {0.03, 0.02, 9.81};
  data.linear_acc = data.linear_acc / data.linear_acc.norm() * 9.81;
  data.angular_vel = {0.0, 0.0, 0.01};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.linear_acc = {0.03, 0.02, 9.81};
  data.linear_acc = data.linear_acc / data.linear_acc.norm() * 9.81;
  data.angular_vel = {0.0, 0.0, 0.01};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.linear_acc = {0.03, 0.05, 9.81};
  data.linear_acc = data.linear_acc / data.linear_acc.norm() * 9.81;
  data.angular_vel = {0.0, 0.0, 0.02};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.linear_acc = {0.03, 0.07, 9.81};
  data.linear_acc = data.linear_acc / data.linear_acc.norm() * 9.81;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.linear_acc = {0.03, 0.0, 9.81};
  data.linear_acc = data.linear_acc / data.linear_acc.norm() * 9.81;
  data.angular_vel = {0.0, 0.0, 0.08};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.linear_acc = {-0.03, 0.0, 9.81};
  data.linear_acc = data.linear_acc / data.linear_acc.norm() * 9.81;
  data.angular_vel = {0.0, 0.0, 0.12};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.linear_acc = {-0.03, 0.0, 9.81};
  data.linear_acc = data.linear_acc / data.linear_acc.norm() * 9.81;
  data.angular_vel = {0.0, 0.0, 0.12};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.linear_acc = {-0.03, -0.01, 9.81};
  data.linear_acc = data.linear_acc / data.linear_acc.norm() * 9.81;
  data.angular_vel = {0.0, 0.0, 0.13};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.14};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.12};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.10};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.09};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.09};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.08};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.08};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.08};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.03};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.01};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.0};
  imu_queue_raw.push_back(data);
  data.time += 0.005;
  data.angular_vel = {0.0, 0.0, 0.0};
  imu_queue_raw.push_back(data);

  imu_preintegrator::ImuPreintegrator imu_preint_true(parameters,
                                                      {{0, 0, 0}, {0, 0, 0}});
  imu_preint_true.InitializePoseAndVelocity(0.0, Eigen::Vector3d::Zero(),
                                            Eigen::Matrix3d::Identity(),
                                            Eigen::Vector3d::Zero());
  for (const auto& d : imu_queue_raw) imu_preint_true.Propagate(d);
  const auto imu_factor_true = imu_preint_true.GetImuFactor();
  std::cerr << "Raw:\n";
  std::cerr << "tij:\n" << imu_factor_true.tij << "\n";
  std::cerr << "R:\n" << imu_factor_true.delRij << "\n";
  std::cerr << "p:" << imu_factor_true.delpij.transpose() << "\n";
  std::cerr << "v:" << imu_factor_true.delvij.transpose() << "\n";
  std::cerr << std::endl;

  imu_preintegrator::ImuPreintegrator imu_preint_biased(parameters,
                                                        {{0, 0, 0}, {0, 0, 0}});
  imu_preint_biased.InitializePoseAndVelocity(0.0, Eigen::Vector3d::Zero(),
                                              Eigen::Matrix3d::Identity(),
                                              Eigen::Vector3d::Zero());
  for (auto& meas : imu_queue_raw) {
    meas.linear_acc += initial_bias.linear_acc;
    meas.angular_vel += initial_bias.angular_vel;
  }
  for (const auto& meas : imu_queue_raw) imu_preint_biased.Propagate(meas);

  const auto imu_factor_biased = imu_preint_biased.GetImuFactor();
  std::cerr << "Biased:\n";
  std::cerr << "tij:\n" << imu_factor_biased.tij << "\n";
  std::cerr << "R:\n" << imu_factor_biased.delRij << "\n";
  std::cerr << "p:" << imu_factor_biased.delpij.transpose() << "\n";
  std::cerr << "v:" << imu_factor_biased.delvij.transpose() << "\n";
  std::cerr << std::endl;

  auto imu_state_true = imu_preint_true.GetImuState();
  auto imu_state_unbiased = imu_preint_biased.GetImuState();
  std::cerr << "ImuState comparison(before):\n";
  std::cerr << imu_state_true.p.transpose() << "/"
            << imu_state_unbiased.p.transpose() << "\n";

  // Updated
  imu_preint_biased.CorrectImuFactorByUpdatedBias(initial_bias);
  imu_preint_biased.RepropagateImuState();

  const auto imu_factor_unbiased = imu_preint_biased.GetImuFactor();
  std::cerr << "Unbiased:\n";
  std::cerr << "tij:\n" << imu_factor_unbiased.tij << "\n";
  std::cerr << "R:\n" << imu_factor_unbiased.delRij << "\n";
  std::cerr << "p:" << imu_factor_unbiased.delpij.transpose() << "\n";
  std::cerr << "v:" << imu_factor_unbiased.delvij.transpose() << "\n";
  std::cerr << std::endl;

  imu_state_unbiased = imu_preint_biased.GetImuState();

  std::cerr << "ImuState comparison (after):\n";
  std::cerr << imu_state_true.p.transpose() << "/"
            << imu_state_unbiased.p.transpose() << "\n";

  return 0;
}
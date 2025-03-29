#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include "Eigen/Dense"

#include "ceres/ceres.h"
#include "flann/algorithms/dist.h"
#include "flann/algorithms/kdtree_single_index.h"
#include "flann/flann.hpp"

#include "optimizer/mahalanobis_distance_minimizer/ceres_cost_functor.h"
#include "optimizer/mahalanobis_distance_minimizer/mahalanobis_distance_minimizer.h"
#include "optimizer/mahalanobis_distance_minimizer/mahalanobis_distance_minimizer_ceres.h"
#include "types.h"

using VoxelKey = uint64_t;
using NdtMap = std::unordered_map<VoxelKey, NDT>;

std::vector<Eigen::Vector3d> GenerateGlobalPoints();
std::vector<Vec3> FilterPoints(const std::vector<Vec3>& points,
                               const double filter_voxel_size);
std::vector<Vec3> WarpPoints(const std::vector<Vec3>& points, const Pose& pose);
void UpdateNdtMap(const std::vector<Vec3>& points,
                  const double voxel_resolution, NdtMap* ndt_map);
VoxelKey ComputeVoxelKey(const Vec3& point,
                         const double inverse_voxel_resolution);
std::vector<Correspondence> MatchPointCloud(
    const NdtMap& ndt_map, const std::vector<Vec3>& local_points,
    const Pose& pose);
Pose OptimizePose(const NdtMap& ndt_map, const std::vector<Vec3>& local_points,
                  const Pose& pose);

int main(int argc, char** argv) {
  constexpr double kVoxelResolution{0.5};
  constexpr double kFilterVoxelResolution{0.1};

  // Make global points
  const auto points = GenerateGlobalPoints();
  std::cerr << "# points: " << points.size() << std::endl;

  // Create NDT map by global points
  NdtMap ndt_map;
  UpdateNdtMap(points, kVoxelResolution, &ndt_map);
  std::cerr << "Ndt map size: " << ndt_map.size() << std::endl;

  // Set true pose
  Pose true_pose{Pose::Identity()};
  true_pose.translation() = Vec3(-0.2, 0.123, 0.3);
  true_pose.linear() =
      Eigen::AngleAxisd(0.05, Vec3(0.0, 0.0, 1.0)).toRotationMatrix();

  // Create local points
  const auto filtered_points = FilterPoints(points, kFilterVoxelResolution);
  const auto local_points = WarpPoints(filtered_points, true_pose.inverse());

  // Optimize pose
  Pose initial_pose{Pose::Identity()};
  const auto optimized_pose = OptimizePose(ndt_map, local_points, initial_pose);

  std::cerr << "Optimized pose: " << optimized_pose.translation().transpose()
            << " "
            << Eigen::Quaterniond(optimized_pose.linear()).coeffs().transpose()
            << std::endl;
  std::cerr << "True pose: " << true_pose.translation().transpose() << " "
            << Eigen::Quaterniond(true_pose.linear()).coeffs().transpose()
            << std::endl;

  return 0;
}

std::vector<Eigen::Vector3d> GenerateGlobalPoints() {
  const double width = 5.0;
  const double length = 7.0;
  const double height = 2.5;
  const double point_step = 0.01;

  std::vector<Eigen::Vector3d> points;

  // floor
  double x, y, z;
  z = 0.0;
  for (x = -length / 2.0; x <= length / 2.0; x += point_step)
    for (y = -width / 2.0; y <= width / 2.0; y += point_step)
      points.push_back(Eigen::Vector3d(x, y, z));

  // left/right wall
  y = -width / 2.0;
  for (x = -length / 2.0; x <= length / 2.0; x += point_step) {
    for (z = 0.0; z <= height; z += point_step) {
      points.push_back(Eigen::Vector3d(x, y, z));
      points.push_back(Eigen::Vector3d(x, -y, z));
    }
  }

  // front/back wall
  x = -length / 2.0;
  for (y = -width / 2.0; y <= width / 2.0; y += point_step) {
    for (z = 0.0; z <= height; z += point_step) {
      points.push_back(Eigen::Vector3d(-x, y, z));
      points.push_back(Eigen::Vector3d(x, y, z));
    }
  }

  return points;
}

std::vector<Vec3> FilterPoints(const std::vector<Vec3>& points,
                               const double filter_voxel_size) {
  const double voxel_resolution{filter_voxel_size};
  const double inv_res = 1.0 / voxel_resolution;
  std::unordered_set<VoxelKey> filtered_voxel_key_set;
  std::vector<Vec3> filtered_points;
  filtered_points.reserve(points.size());
  for (const auto& point : points) {
    const auto voxel_key = ComputeVoxelKey(point, inv_res);
    if (filtered_voxel_key_set.find(voxel_key) !=
        filtered_voxel_key_set.end()) {
      continue;
    }
    filtered_voxel_key_set.insert(voxel_key);
    filtered_points.push_back(point);
  }
  return filtered_points;
}

std::vector<Vec3> WarpPoints(const std::vector<Vec3>& points,
                             const Pose& pose) {
  std::vector<Vec3> warped_points;
  warped_points.reserve(points.size());
  for (const auto& point : points) {
    Vec3 warped_point = pose * point;
    warped_points.push_back(warped_point);
  }
  return warped_points;
}

void UpdateNdtMap(const std::vector<Vec3>& points,
                  const double voxel_resolution, NdtMap* ndt_map) {
  const double inv_res = 1.0 / voxel_resolution;

  std::unordered_set<VoxelKey> updated_voxel_key_set;
  for (const auto& point : points) {
    const auto voxel_key = ComputeVoxelKey(point, inv_res);
    updated_voxel_key_set.insert(voxel_key);

    auto& ndt = (*ndt_map)[voxel_key];
    ++ndt.count;
    ndt.sum += point;
    ndt.moment += point * point.transpose();
  }

  for (const auto& voxel_key : updated_voxel_key_set) {
    auto& ndt = ndt_map->at(voxel_key);
    if (ndt.count < 5) continue;

    ndt.mean = ndt.sum / ndt.count;
    const Mat3x3 covariance =
        ndt.moment / ndt.count - ndt.mean * ndt.mean.transpose();

    Eigen::SelfAdjointEigenSolver<Mat3x3> eigsol(covariance);
    if (eigsol.info() != Eigen::Success || eigsol.eigenvalues()(2) < 0.01) {
      ndt.is_valid = false;
      return;
    }

    const double min_eigval_ratio = 0.01;

    ndt.is_valid = true;
    Vec3 eigvals = eigsol.eigenvalues();
    eigvals(0) = std::max(eigvals(0), eigvals(2) * min_eigval_ratio);
    eigvals(1) = std::max(eigvals(1), eigvals(2) * min_eigval_ratio);

    ndt.sqrt_information = eigvals.cwiseSqrt().cwiseInverse().asDiagonal() *
                           eigsol.eigenvectors().transpose();

    ndt.information = ndt.sqrt_information.transpose() * ndt.sqrt_information;
  }
}

VoxelKey ComputeVoxelKey(const Vec3& point,
                         const double inverse_voxel_resolution) {
  int x_key = std::floor(point.x() * inverse_voxel_resolution);
  int y_key = std::floor(point.y() * inverse_voxel_resolution);
  int z_key = std::floor(point.z() * inverse_voxel_resolution);
  x_key = x_key > 0 ? 2 * x_key : -2 * x_key - 1;
  y_key = y_key > 0 ? 2 * y_key : -2 * y_key - 1;
  z_key = z_key > 0 ? 2 * z_key : -2 * z_key - 1;
  uint64_t xy_key = (x_key + y_key) * (x_key + y_key + 1) / 2 + y_key;
  uint64_t xyz_key = (xy_key + z_key) * (xy_key + z_key + 1) / 2 + z_key;
  return xyz_key;
}

std::vector<Correspondence> MatchPointCloud(
    const NdtMap& ndt_map, const std::vector<Vec3>& local_points,
    const Pose& pose) {
  constexpr int kNumNeighbors{3};
  constexpr double kSearchRadius{1.0};

  const auto points = WarpPoints(local_points, pose);

  // Generate kdtree of the voxel_key-ndt mean pair
  flann::KDTreeSingleIndex<flann::L2_Simple<double>> kdtree;
  std::vector<VoxelKey> voxel_keys;
  std::vector<Vec3> ndt_means;
  for (const auto& [voxel_key, ndt] : ndt_map) {
    if (!ndt.is_valid) continue;
    voxel_keys.push_back(voxel_key);
    ndt_means.push_back(ndt.mean);
  }

  double* points_array = const_cast<double*>(ndt_means.data()->data());
  auto flann_points = flann::Matrix<double>(points_array, ndt_means.size(), 3);
  kdtree.buildIndex(flann_points);

  // Find the nearest neighbors in the kdtree
  std::vector<Correspondence> correspondences;
  correspondences.reserve(points.size());
  flann::Matrix<double> flann_query(const_cast<double*>(points.data()->data()),
                                    points.size(), 3);

  std::cerr << flann_query.rows << " " << flann_query.cols << std::endl;

  std::vector<std::vector<size_t>> indices;
  std::vector<std::vector<double>> distances;
  indices.resize(points.size());
  distances.resize(points.size());
  auto search_params = flann::SearchParams();
  search_params.max_neighbors = kNumNeighbors;
  search_params.eps = 0.2;
  if (kdtree.radiusSearch(flann_query, indices, distances,
                          kSearchRadius * kSearchRadius, search_params)) {
    for (size_t point_index = 0; point_index < points.size(); ++point_index) {
      const auto& indices_of_point = indices[point_index];
      for (const auto& index : indices_of_point) {
        const auto& matched_ndt = ndt_map.at(voxel_keys[index]);
        correspondences.emplace_back(
            Correspondence{points.at(point_index), matched_ndt});
      }
    }
  }

  return correspondences;
}

Pose OptimizePose(const NdtMap& ndt_map, const std::vector<Vec3>& local_points,
                  const Pose& initial_pose) {
  double optimized_translation[3] = {initial_pose.translation().x(),
                                     initial_pose.translation().y(),
                                     initial_pose.translation().z()};
  Orientation optimized_orientation(initial_pose.rotation());

  Pose optimized_pose{Pose::Identity()};
  for (int outer_iter = 0; outer_iter < 10; ++outer_iter) {
    Pose last_optimized_pose{Pose::Identity()};
    last_optimized_pose.translation() =
        Vec3(optimized_translation[0], optimized_translation[1],
             optimized_translation[2]);
    last_optimized_pose.linear() =
        Eigen::Quaterniond(optimized_orientation).toRotationMatrix();

    // Find correspondences
    const auto correspondences =
        MatchPointCloud(ndt_map, local_points, last_optimized_pose);

    ceres::Problem problem;
    ceres::CostFunction* cost_function =
        optimizer::mahalanobis_distance_minimizer::
            RedundantMahalanobisDistanceCostFunctor::Create(correspondences,
                                                            1.0, 1.0);
    problem.AddResidualBlock(cost_function, nullptr, optimized_translation,
                             optimized_orientation.coeffs().data());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 20;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cerr << "Summary: " << summary.BriefReport() << std::endl;

    Pose current_optimized_pose{Pose::Identity()};
    current_optimized_pose.translation() =
        Vec3(optimized_translation[0], optimized_translation[1],
             optimized_translation[2]);
    current_optimized_pose.linear() = optimized_orientation.toRotationMatrix();
    optimized_pose = current_optimized_pose;
    Pose pose_diff = current_optimized_pose.inverse() * last_optimized_pose;
    if (pose_diff.translation().norm() < 1e-5 &&
        Orientation(pose_diff.linear()).vec().norm() < 1e-5) {
      break;
    }
  }

  return optimized_pose;
}
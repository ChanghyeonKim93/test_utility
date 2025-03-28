#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include "Eigen/Dense"

#include "nanoflann.h"
#include "types.h"

using VoxelKey = uint64_t;
using NdtMap = std::unordered_map<VoxelKey, NDT>;

std::vector<Eigen::Vector3d> GeneratePointCloud();
void UpdateNdtMap(const std::vector<Vec3>& points, NdtMap* ndt_map);
VoxelKey ComputeVoxelKey(const Vec3& point,
                         const double inverse_voxel_resolution);
std::vector<Correspondence> MatchPointCloud(const NdtMap& ndt_map,
                                            const std::vector<Vec3>& points);

int main(int argc, char** argv) {
  const auto points = GeneratePointCloud();
  std::cerr << "# points: " << points.size() << std::endl;

  NdtMap ndt_map;
  UpdateNdtMap(points, &ndt_map);
  std::cerr << "Ndt map size: " << ndt_map.size() << std::endl;

  return 0;
}

std::vector<Eigen::Vector3d> GeneratePointCloud() {
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

void UpdateNdtMap(const std::vector<Vec3>& points, NdtMap* ndt_map) {
  const double voxel_resolution{0.5};
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
    const auto covariance =
        ndt.moment / ndt.count - ndt.mean * ndt.mean.transpose();

    Eigen::SelfAdjointEigenSolver<Mat3x3> eigsol(covariance);
    if (eigsol.info() != Eigen::Success || eigsol.eigenvalues()(2) < 0.1) {
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

std::vector<Correspondence> MatchPointCloud(const NdtMap& ndt_map,
                                            const std::vector<Vec3>& points) {
  std::vector<Correspondence> correspondences;
  correspondences.reserve(points.size());
  for (const auto& point : points) {
    const auto voxel_key = ComputeVoxelKey(point, 1.0 / 0.5);
    if (ndt_map.find(voxel_key) == ndt_map.end()) continue;

    const auto& ndt = ndt_map.at(voxel_key);
    if (!ndt.is_valid) continue;

    Eigen::Vector3d transformed_point = ndt.sqrt_information * point;
    correspondences.emplace_back(Correspondence{point, ndt});
  }
  // I want to use flann::KDTreeSingleIndex to find the nearest neighbors
  // for (const auto& point : points) {
  //   const auto voxel_key = ComputeVoxelKey(point, 1.0 / 0.5);
  //   if (ndt_map.find(voxel_key) == ndt_map.end()) continue;
  //   const auto& ndt = ndt_map.at(voxel_key);
  //   if (!ndt.is_valid) continue;
  //   const auto& mean_point = ndt.mean;
  //   const auto& covariance = ndt.covariance;
  //   const auto& sqrt_information = ndt.sqrt_information;
  //   const auto& information = ndt.information;
  //   const auto& transformed_point = ndt.sqrt_information * point;
  //   correspondences.emplace_back(Correspondence{point, transformed_point});
  // }

  return correspondences;
}
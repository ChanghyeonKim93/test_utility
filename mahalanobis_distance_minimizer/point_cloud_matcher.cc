

#include <chrono>
#include <random>
#include <unordered_set>

#include "depth_localizer/map_builder/point_cloud_matcher.h"

namespace depth_localizer {
namespace map_builder {
namespace point_cloud_matcher {

PointCloudMatcher::PointCloudMatcher(const Parameters &parameters)
    : parameters_(parameters) {}

std::vector<Correspondence> PointCloudMatcher::MatchPointCloud(
    const std::unordered_map<uint64_t, NormalDistribution> &ndt_map,
    const std::vector<Eigen::Vector3d> &point_cloud_data, const Pose &pose) {
  std::vector<Eigen::Vector3d> mean_point_list;
  mean_point_list.reserve(ndt_map.size());
  for (const auto &[key, ndt] : ndt_map)
    mean_point_list.push_back(ndt.mean);
  const auto kd_tree = GenerateKdTree(mean_point_list);

  std::vector<Correspondence> correspondence_list;
  const auto num_neighbors = parameters_.correspondence.num_neighbors;
  correspondence_list.reserve(point_cloud_data.size() * num_neighbors);
  for (const auto &point_in_baselink : point_cloud_data) {
    const auto query_point_in_local_map = pose * point_in_baselink;
    const auto neighbor_ndt_mean_list = SearchNeighborNdtMeanList(
        query_point_in_local_map, mean_point_list, num_neighbors, kd_tree);
    if (neighbor_ndt_mean_list.empty())
      continue;
    for (const auto &ndt_mean : neighbor_ndt_mean_list) {
      const auto voxel_key = GetVoxelKey(ndt_mean, parameters_.ndt_resolution);
      if (ndt_map.find(voxel_key) == ndt_map.end())
        continue;
      const auto &ndt = ndt_map.at(voxel_key);
      if (ndt.is_singular)
        continue;
      correspondence_list.push_back({ndt, point_in_baselink});
    }
  }

  return parameters_.ransac.enabled
             ? FilterOutlierCorrespondences(correspondence_list)
             : correspondence_list;
}

flann::KDTreeSingleIndex<flann::L2_Simple<double>>
PointCloudMatcher::GenerateKdTree(
    const std::vector<Eigen::Vector3d> &mean_point_list) {
  flann::KDTreeSingleIndex<flann::L2_Simple<double>> kd_tree;
  double *points_array = const_cast<double *>(mean_point_list.data()->data());
  auto flann_points =
      flann::Matrix<double>(points_array, mean_point_list.size(), 3);
  kd_tree.buildIndex(flann_points);
  return kd_tree;
}

std::vector<Eigen::Vector3d> PointCloudMatcher::SearchNeighborNdtMeanList(
    const Eigen::Vector3d &query_point,
    const std::vector<Eigen::Vector3d> &mean_point_list,
    const double num_neighbors,
    const flann::KDTreeSingleIndex<flann::L2_Simple<double>> &kd_tree) {
  std::vector<Eigen::Vector3d> neighbor_ndt_mean_list;

  std::vector<std::vector<size_t>> indices;
  std::vector<std::vector<double>> distance_list;
  Eigen::Vector3d query = query_point;
  const double search_radius =
      parameters_.ndt_resolution * parameters_.ndt_resolution;

  auto search_params = flann::SearchParams();
  search_params.max_neighbors = num_neighbors;
  if (kd_tree.radiusSearch(flann::Matrix<double>(query.data(), 1, 3), indices,
                           distance_list, search_radius, search_params)) {
    for (const auto &index : indices[0]) {
      auto neighbor_ndt_mean = mean_point_list[index];
      neighbor_ndt_mean_list.push_back(neighbor_ndt_mean);
    }
  }
  return neighbor_ndt_mean_list;
}

uint64_t PointCloudMatcher::GetVoxelKey(const Eigen::Vector3d &point,
                                        const double voxel_size) {
  Eigen::Vector3i point_index =
      (point.array() / voxel_size).round().cast<int>();
  const auto x = static_cast<int64_t>(point_index.x());
  const auto y = static_cast<int64_t>(point_index.y());
  const auto z = static_cast<int64_t>(point_index.z());
  const auto key_x = (x >= 0) ? (x * 2) : (x * (-2) - 1);
  const auto key_y = (y >= 0) ? (y * 2) : (y * (-2) - 1);
  const auto key_z = (z >= 0) ? (z * 2) : (z * (-2) - 1);
  const auto key_xy = (key_x + key_y) * (key_x + key_y + 1) / 2 + key_y;
  const auto voxel_key = (key_xy + key_z) * (key_xy + key_z + 1) / 2 + key_z;
  return static_cast<uint64_t>(voxel_key);
}

std::vector<Correspondence> PointCloudMatcher::FilterOutlierCorrespondences(
    const std::vector<Correspondence> &correspondence_list) {
  const auto sample_size = parameters_.ransac.sample_size;
  if (correspondence_list.size() < static_cast<size_t>(sample_size))
    return {};

  const auto num_iterations = parameters_.ransac.num_iterations;
  const auto squared_inlier_threshold =
      parameters_.ransac.inlier_threshold * parameters_.ransac.inlier_threshold;
  const auto desired_inlier_probability =
      parameters_.ransac.desired_inlier_probability;

  std::vector<Correspondence> best_inlier_correspondence_list;
  std::vector<int> best_inlier_correspondence_index_list;
  static std::random_device random_device;
  std::mt19937 random_number(random_device());
  std::uniform_int_distribution<int> uniform_distribution(
      0, correspondence_list.size());
  for (int iteration = 0; iteration < num_iterations; iteration++) {
    std::unordered_set<int> sampled_indices;
    while (sampled_indices.size() < static_cast<size_t>(sample_size)) {
      const auto index = uniform_distribution(random_number);
      if (sampled_indices.find(index) == sampled_indices.end())
        sampled_indices.insert(index);
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> source(3, sample_size);
    Eigen::Matrix<double, 3, Eigen::Dynamic> target(3, sample_size);
    auto index_iterator = sampled_indices.begin();
    for (int i = 0; i < sample_size; i++) {
      const auto index = *index_iterator++;
      source.block<3, 1>(0, i) = correspondence_list[index].point;
      target.block<3, 1>(0, i) =
          correspondence_list[index].normal_distribution.mean;
    }

    Eigen::Matrix4d transformation_matrix =
        Eigen::umeyama(source, target, false);
    const auto rotation = transformation_matrix.block<3, 3>(0, 0);
    const auto translation = transformation_matrix.block<3, 1>(0, 3);
    std::vector<int> inlier_index_list;
    int correspondence_list_size = static_cast<int>(correspondence_list.size());
    for (int i = 0; i < correspondence_list_size; i++) {
      const auto &source_point = correspondence_list[i].point;
      const auto &target_point =
          correspondence_list[i].normal_distribution.mean;
      const auto transformed_point = rotation * source_point + translation;
      const auto squared_distance =
          (target_point - transformed_point).squaredNorm();
      if (squared_distance < squared_inlier_threshold)
        inlier_index_list.push_back(i);
    }

    if (inlier_index_list.size() > best_inlier_correspondence_index_list.size())
      best_inlier_correspondence_index_list = inlier_index_list;

    const int num_best_inlier_correspondences =
        static_cast<int>(best_inlier_correspondence_index_list.size());
    const int num_correspondences =
        static_cast<int>(correspondence_list.size());
    const auto inlier_probability = num_best_inlier_correspondences /
                                    static_cast<double>(num_correspondences);
    if (inlier_probability >= desired_inlier_probability)
      break;
  }

  best_inlier_correspondence_list.reserve(
      best_inlier_correspondence_index_list.size());
  for (const auto index : best_inlier_correspondence_index_list)
    best_inlier_correspondence_list.push_back(correspondence_list[index]);

  return best_inlier_correspondence_list;
}

} // namespace point_cloud_matcher
} // namespace map_builder
} // namespace depth_localizer

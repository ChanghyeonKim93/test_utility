#ifndef DEPTH_LOCALIZER_MAP_BUILDER_POINT_CLOUD_MATCHER_H_
#define DEPTH_LOCALIZER_MAP_BUILDER_POINT_CLOUD_MATCHER_H_

#include <unordered_map>
#include <vector>

#include "Eigen/Dense"
#include "flann/algorithms/dist.h"
#include "flann/algorithms/kdtree_single_index.h"

#include "depth_localizer/map_builder/types.h"

namespace depth_localizer {
namespace map_builder {
namespace point_cloud_matcher {

struct Parameters {
  double ndt_resolution{0.5};
  struct {
    bool enabled{false};
    int num_iterations{12};
    int sample_size{5};
    double inlier_threshold{0.5};  // [m]
    double desired_inlier_probability{0.85};
  } ransac;
  struct {
    int num_neighbors{5};
  } correspondence;
};

class PointCloudMatcher {
 public:
  /// @brief Constructor
  /// @param[in] parameters configuration parameters
  explicit PointCloudMatcher(const Parameters& parameters);

  /// @brief Match point cloud with given ndt_map
  /// @param[in] ndt_map ndt_map to match point_cloud with
  /// @param[in] point_cloud_data point cloud data to match with ndt_map
  /// @param[in] pose pose of point cloud represented in ndt map frame
  /// @return Correspondence list
  std::vector<Correspondence> MatchPointCloud(
      const std::unordered_map<uint64_t, NormalDistribution>& ndt_map,
      const std::vector<Eigen::Vector3d>& point_cloud_data, const Pose& pose);

 private:
  flann::KDTreeSingleIndex<flann::L2_Simple<double>> GenerateKdTree(
      const std::vector<Eigen::Vector3d>& mean_point_list);
  std::vector<Eigen::Vector3d> SearchNeighborNdtMeanList(
      const Eigen::Vector3d& query_point,
      const std::vector<Eigen::Vector3d>& mean_point_list,
      const double num_neighbors,
      const flann::KDTreeSingleIndex<flann::L2_Simple<double>>& kd_tree);
  uint64_t GetVoxelKey(const Eigen::Vector3d& point, const double voxel_size);
  std::vector<Correspondence> FilterOutlierCorrespondences(
      const std::vector<Correspondence>& correspondence_list);

  const Parameters parameters_;

  friend class PointCloudMatcherTest_GenerateCorrespondenceListTest_Test;
  friend class PointCloudMatcherTest_GenerateMultiCorrespondenceListTest_Test;
  friend class PointCloudMatcherTest_FilterOutlierCorrespondencesTest_Test;
};

}  // namespace point_cloud_matcher
}  // namespace map_builder
}  // namespace depth_localizer

#endif  // DEPTH_LOCALIZER_MAP_BUILDER_POINT_CLOUD_MATCHER_H_

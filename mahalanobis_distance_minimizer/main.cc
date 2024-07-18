#include <any>
#include <iostream>
#include <string>
#include <type_traits>
#include <unordered_set>

#include "Eigen/Dense"

#include "time_checker.h"

class ParameterBase {
 public:
  enum class Type { kUnknown, kBool, kInt, kDouble, kString, kFootprint };
  struct Footprint {
    enum class Type { kUnknown, kRectangular, kCircular, kPolygon };
    Type type{Type::kUnknown};
    double height{0.0};
    double length{0.0};
    double width{0.0};
    double radius{0.0};
    struct {
      double x{0.0};
      double y{0.0};
    } offset;
  };

  std::string key_{""};
  Type type_{Type::kUnknown};
  std::any value_;

  template <typename Dst>
  Dst ConvertTo() {
    Dst dst;
    dst.key = key_;
    dst.type = type_;
    dst.value = value_;
    return dst;
  }
};

namespace motion_planner::forward_planner {

class Parameter : public ParameterBase {};

}  // namespace motion_planner::forward_planner

namespace motion_planner::forward_planner::bridge {

class Parameter : public ParameterBase {};

}  // namespace motion_planner::forward_planner::bridge

namespace behavior_planner {

class Parameter : public ParameterBase {};

}  // namespace behavior_planner
namespace behavior_planner::bridge {

class Parameter : public ParameterBase {};

}  // namespace behavior_planner::bridge

struct NormalDistribution {
  int count{0};
  Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d moment{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d sqrt_information{Eigen::Matrix3d::Zero()};

  void AddPoint(const Eigen::Vector3d& point) {
    ++count;
    mean += point;
    moment += point * point.transpose();
  }

  void Finalize() {
    if (count < 3) return;
  }
};

struct Correspondence {
  int normal_distribution_index{0};
  int point_index{0};
};

class MahalanobisDistanceMinimizer {
 public:
  MahalanobisDistanceMinimizer();

  void RegisterNormalDistribution(
      const NormalDistribution& normal_distribution);
  void RegisterPointInReferenceFrame(
      const Eigen::Vector3d& point_in_reference_frame);

  void ResetCorrespondenceList();

  void GetNormalDistributionList() const;
  void GetPoitnList() const;

  void AddCorrespodence(const int normal_distribution_index,
                        const int point_index);

  virtual bool Solve(Eigen::Isometry3d* pose) = 0;

 private:
  std::vector<Correspondence> correspondence_list_;
  std::vector<NormalDistribution> normal_distribution_list_;
  std::vector<Eigen::Vector3d> point_list_;
};

MahalanobisDistanceMinimizer::MahalanobisDistanceMinimizer() {}

void MahalanobisDistanceMinimizer::RegisterNormalDistribution(
    const NormalDistribution& normal_distribution) {
  normal_distribution_list_.push_back(normal_distribution);
}

void MahalanobisDistanceMinimizer::RegisterPointInReferenceFrame(
    const Eigen::Vector3d& point) {
  point_list_.push_back(point);
}

void MahalanobisDistanceMinimizer::AddCorrespodence(
    const int normal_distribution_index, const int point_index) {
  correspondence_list_.push_back({normal_distribution_index, point_index});
}

void MahalanobisDistanceMinimizer::ResetCorrespondenceList() {
  correspondence_list_.clear();
}

int main() {
  ParameterBase::Footprint footprint;

  motion_planner::forward_planner::bridge::Parameter fp_bridge_param;
  fp_bridge_param = {"a", ParameterBase::Type::kBool, 1.2};

  motion_planner::forward_planner::Parameter fp_param;
  auto a = motion_planner::forward_planner::Parameter{
      "max_linear_velocity", ParameterBase::Type::kDouble, 1.65};

  return 0;
}
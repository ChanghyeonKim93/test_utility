#include <iostream>

#include "Eigen/Dense"

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

int main() { return 0; }
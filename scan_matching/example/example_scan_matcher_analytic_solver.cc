#include <iostream>
#include <memory>
#include <vector>

#include "scan_matcher.h"
#include "scan_matcher_analytic_solver.h"

#include "eigen3/Eigen/Dense"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

int CalculateIndex(const double x, const double y, const int num_cols,
                   const int num_rows, const double resolution) {
  const int row = std::floor(y / resolution);
  const int col = std::floor(x / resolution);
  return row * num_cols + col;
}

int main() {
  std::unique_ptr<ScanMatcher> scan_matcher =
      std::make_unique<ScanMatcherAnalyticSolver>();

  constexpr double map_resolution{0.05};  // [m]
  constexpr int map_columns{200};
  constexpr int map_rows{200};

  std::vector<uint8_t> probability_map(map_columns * map_rows, 0);

  // wall points
  constexpr double step_size = 0.02;
  std::vector<Eigen::Vector2d> corner_point_list{
      {2.0, 1.0}, {2.0, 6.0}, {5.0, 6.0}, {5.0, 2.5}, {8.5, 2.5}};
  std::vector<Eigen::Vector2d> true_scan_point_list;
  auto it = corner_point_list.begin();
  auto it_next = it + 1;
  auto it_back = corner_point_list.end() - 1;
  for (; it != it_back; ++it, ++it_next) {
    const auto diff_vector = *it_next - *it;
    const auto unit_vector = diff_vector / diff_vector.norm();
    const int num_steps = std::floor(diff_vector.norm() / step_size);
    std::cout << num_steps << std::endl;
    for (int i = 0; i < num_steps; ++i) {
      auto pt = *it + step_size * i * unit_vector;
      true_scan_point_list.push_back(pt);
      std::cerr << pt << std::endl;
    }
  }
  std::cerr << true_scan_point_list.size() << std::endl;
  cv::Mat probability_map_image =
      cv::Mat::zeros(cv::Size(map_rows, map_columns), CV_8UC1);
  for (const auto& pt : true_scan_point_list) {
    const int index =
        CalculateIndex(pt.x(), pt.y(), map_columns, map_rows, map_resolution);
    *(probability_map_image.data + index) = 255;
  }

  for (int iter = 0; iter < 7; ++iter)
    cv::GaussianBlur(probability_map_image, probability_map_image,
                     cv::Size(3, 3), 0.5);
  for (const auto& pt : true_scan_point_list) {
    const int index =
        CalculateIndex(pt.x(), pt.y(), map_columns, map_rows, map_resolution);
    *(probability_map_image.data + index) = 255;
  }

  cv::namedWindow("image");
  cv::imshow("image", probability_map_image);
  cv::waitKey(0);

  // Generate pose
  Eigen::Vector3d pose_parameters{1.0, 2.24, 0.21};
  Eigen::Isometry2d pose{Eigen::Isometry2d::Identity()};
  pose.translation().x() = pose_parameters(0);
  pose.translation().y() = pose_parameters(1);
  pose.linear() = Eigen::Rotation2Dd(pose_parameters(2)).toRotationMatrix();

  const auto inverse_pose = pose.inverse();

  std::vector<Eigen::Vector2d> scan_point_list;
  for (const auto& pt : true_scan_point_list)
    scan_point_list.push_back(inverse_pose * pt);

  return 0;
}
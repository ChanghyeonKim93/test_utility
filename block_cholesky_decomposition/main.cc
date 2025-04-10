#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/SparseCholesky"

#include "block_cholesky_decomposer.h"
#include "timer.h"

using namespace std::chrono_literals;

int main() {
  const int kDimRes{2};
  const int kNumRes{1900};
  const int kDimParam(2);
  const int kNumParam{600};

  using BlockMatrix = Eigen::Matrix<double, kDimParam, kDimParam>;
  using BlockVector = Eigen::Matrix<double, kDimParam, 1>;

  Eigen::MatrixXd jacobian(kDimRes * kNumRes, kDimParam * kNumParam);
  for (int n = 0; n < kNumRes; ++n) {
    for (int m = 0; m < kNumParam; ++m) {
      jacobian.block<kDimRes, kDimParam>(kDimRes * n, kDimParam * m) =
          Eigen::Matrix<double, kDimRes, kDimParam>::Random();
    }
  }
  Eigen::MatrixXd H_true(kDimParam * kNumParam, kDimParam * kNumParam);
  H_true = jacobian.transpose() * jacobian;
  std::cerr << "H_true.determinant(): " << (H_true.determinant()) << std::endl;
  std::cerr << "det(H) > 0: " << (H_true.determinant() > 0.0) << std::endl;

  Eigen::MatrixXd g_true(kDimParam * kNumParam, 1);
  for (int i = 0; i < kNumParam; ++i)
    g_true.block<kDimParam, 1>(kDimParam * i, 0).setRandom();

  timer::tic();
  const Eigen::MatrixXd x_large_true1 = H_true.inverse() * g_true;
  std::cerr << "Eigen inverse() time is ";
  timer::toc(1);
  // std::cerr << " diff1: " << H_true * x_large_true1 - g_true << std::endl;

  timer::tic();
  Eigen::LDLT<Eigen::MatrixXd> eigen_ldlt;
  auto x_ldlt = eigen_ldlt.compute(H_true).solve(g_true);
  std::cerr << "Eigen LDLt time is ";
  timer::toc(1);
  // std::cerr << " diff2: " << H_true * x_ldlt - g_true << std::endl;

  //
  std::vector<std::vector<BlockMatrix>> H(kNumParam);
  std::vector<BlockVector> g(kNumParam, BlockVector::Zero());
  for (int i = 0; i < kNumParam; ++i) {
    g[i] = g_true.block<kDimParam, 1>(kDimParam * i, 0);
    H.at(i) = std::vector<BlockMatrix>(kNumParam, BlockMatrix::Zero());
    for (int j = 0; j < kNumParam; ++j) {
      H.at(i).at(j) =
          H_true.block<kDimParam, kDimParam>(kDimParam * i, kDimParam * j);
    }
  }

  timer::tic();
  BlockCholeskyDecomposer<double, kDimParam, kNumParam> block_ldlt;
  block_ldlt.DecomposeMatrix(H);
  const auto x = block_ldlt.SolveLinearEquation(g);
  std::cerr << "Block Cholesky time is ";
  timer::toc(1);

  const auto L_est = block_ldlt.GetMatrixL();
  const auto D_est = block_ldlt.GetMatrixD();

  Eigen::MatrixXd L_mat_est(kDimParam * kNumParam, kDimParam * kNumParam);
  Eigen::MatrixXd D_mat_est(kDimParam * kNumParam, kDimParam * kNumParam);
  L_mat_est.setZero();
  D_mat_est.setZero();
  for (int row = 0; row < kNumParam; ++row) {
    D_mat_est.block<kDimParam, kDimParam>(kDimParam * row, kDimParam * row) =
        D_est[row];
    for (int col = 0; col <= row; ++col) {
      L_mat_est.block<kDimParam, kDimParam>(kDimParam * row, kDimParam * col) =
          L_est[row][col];
    }
  }

  // auto A_mat_est = L_mat_est * D_mat_est * L_mat_est.transpose();
  for (int i = 0; i < kNumParam; ++i) {
    double a = (x[i] - x_ldlt.block<kDimParam, 1>(kDimParam * i, 0)).norm();
    // std::cerr << "Norm of " << i << "-th parameter block is "
    //           << (x[i] - x_ldlt.block<kDimParam, 1>(kDimParam * i, 0)).norm()
    //           << std::endl;
  }

  // auto diff_H = H_true - A_mat_est;
  // std::cerr << "DIFF!\n";
  // std::cerr << diff_H << std::endl;

  // Eigen::MatrixXd L_true = eigen_ldlt.compute(H_true).matrixL();
  // std::cerr << "Ltrue\n";
  // std::cerr << L_true - L_mat_est << std::endl;

  return 0;
}
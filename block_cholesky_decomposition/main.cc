
#include <iostream>
#include <vector>

#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/Dense"

#include "block_cholesky_decomposer.h"
#include "timer.h"

using Mat33 = Eigen::Matrix<double, 3, 3>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using BlockFullMat33 = std::vector<std::vector<Mat33>>;
using BlockDiagMat33 = std::vector<Mat33>;
using BlockVector3 = std::vector<Vec3>;

int main() {
  const int kDimensionOfBlock{3};
  const int kNumBlocks{41};
  BlockCholeskyDecomposer<double, kDimensionOfBlock, kNumBlocks> chol_ldlt;

  BlockFullMat33 A(kNumBlocks, std::vector<Mat33>(kNumBlocks));
  for (int row = 0; row < kNumBlocks; ++row) {
    for (int col = 0; col < kNumBlocks; ++col) {
      A[row][col].setZero();
    }
  }

  BlockVector3 b(kNumBlocks);
  for (int row = 0; row < kNumBlocks; ++row) {
    b[row].setRandom();
    for (int col = row; col < kNumBlocks; ++col) {
      Mat33 rand_mat;
      rand_mat.setRandom();
      A[row][col] = rand_mat.transpose() * rand_mat;

      A[col][row] = A[row][col].transpose();
    }
  }

  Eigen::Matrix<double, 3 * kNumBlocks, 3 * kNumBlocks> A_large_true;
  Eigen::Matrix<double, 3 * kNumBlocks, 1> b_large_true;
  for (int row = 0; row < kNumBlocks; ++row) {
    b_large_true.block<3, 1>(3 * row, 0) = b[row];
    for (int col = 0; col < kNumBlocks; ++col) {
      A_large_true.block<3, 3>(3 * row, 3 * col) = A[row][col];
    }
  }
  const Eigen::Matrix<double, 3 * kNumBlocks, 1> x_large_true =
      A_large_true.inverse() * b_large_true;

  timer::tic();
  Eigen::LDLT<Eigen::MatrixXd> ldlt;
  ldlt.compute(A_large_true);
  timer::toc(1);

  BlockFullMat33 L(kNumBlocks, std::vector<Mat33>(kNumBlocks));
  for (int row = 0; row < kNumBlocks; ++row) {
    for (int col = 0; col < kNumBlocks; ++col) {
      L[row][col].setZero();
    }
  }

  BlockDiagMat33 D(kNumBlocks);
  for (int i = 0; i < kNumBlocks; ++i) {
    D[i].setZero();
  }

  for (int j = 0; j < kNumBlocks; j++) {
    Mat33 mat_sum;
    mat_sum.setZero();
    for (int k = 0; k < j; ++k) {
      mat_sum += L[j][k] * D[k] * L[j][k].transpose();
    }
    D[j] = A[j][j] - mat_sum;

    for (int i = j; i < kNumBlocks; ++i) {
      mat_sum.setZero();
      for (int k = 0; k < j; ++k) {
        mat_sum += L[i][k] * D[k] * L[j][k].transpose();
      }

      L[i][j] = (A[i][j] - mat_sum) * D[j].inverse();
    }
  }

  //
  Eigen::Matrix<double, 3 * kNumBlocks, 3 * kNumBlocks> L_mat;
  Eigen::Matrix<double, 3 * kNumBlocks, 3 * kNumBlocks> D_mat;
  L_mat.setZero();
  D_mat.setZero();
  for (int row = 0; row < kNumBlocks; ++row) {
    D_mat.block<3, 3>(3 * row, 3 * row) = D[row];
    for (int col = 0; col <= row; ++col) {
      L_mat.block<3, 3>(3 * row, 3 * col) = L[row][col];
    }
  }

  auto A_large_1 = L_mat * D_mat * L_mat.transpose();
  // std::cerr << "A_large diff:\n" << A_large_true - A_large_1 << std::endl;
  //
  timer::tic();
  const auto x2 = chol_ldlt.DecomposeMatrix(A).SolveLinearEquation(b);
  timer::toc(1);
  const auto L2 = chol_ldlt.GetMatrixL();
  const auto D2 = chol_ldlt.GetMatrixD();

  Eigen::Matrix<double, 3 * kNumBlocks, 3 * kNumBlocks> L_mat2;
  Eigen::Matrix<double, 3 * kNumBlocks, 3 * kNumBlocks> D_mat2;
  L_mat2.setZero();
  D_mat2.setZero();
  for (int row = 0; row < kNumBlocks; ++row) {
    D_mat2.block<3, 3>(3 * row, 3 * row) = D2[row];
    for (int col = 0; col <= row; ++col) {
      L_mat2.block<3, 3>(3 * row, 3 * col) = L2[row][col];
    }
  }

  auto A_large_2 = L_mat2 * D_mat2 * L_mat2.transpose();
  // std::cerr << "A_large diff:\n" << A_large_true - A_large_2 << std::endl;
  // std::cerr << "D_mat diff:\n" << D_mat - D_mat2 << std::endl;
  // for (int i = 0; i < kNumBlocks; ++i) {
  //   std::cerr << (x2[i] - x_large_true.block<3, 1>(3 * i, 0)) << std::endl;
  // }

  return 0;
}
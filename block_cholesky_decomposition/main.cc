#include <iostream>
#include <vector>

#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/Dense"

#include "block_cholesky_decomposer.h"
#include "timer.h"

int main() {
  const int Dim{2};
  const int M{200};

  using BlockMatrix = Eigen::Matrix<double, Dim, Dim>;
  using BlockVector = Eigen::Matrix<double, Dim, 1>;

  std::vector<std::vector<BlockMatrix>> A(M, std::vector<BlockMatrix>(M));
  std::vector<BlockVector> b(M);
  for (int row = 0; row < M; ++row) {
    b[row].setRandom();
    for (int col = 0; col <= row; ++col) {
      BlockMatrix rand_mat;
      rand_mat.setRandom();
      A[row][col] = (rand_mat.transpose() * rand_mat);
      A[col][row].noalias() = A[row][col].transpose();
    }
  }

  Eigen::MatrixXd A_large_true(Dim * M, Dim * M);
  Eigen::MatrixXd b_large_true(Dim * M, 1);
  for (int row = 0; row < M; ++row) {
    b_large_true.block<Dim, 1>(Dim * row, 0) = b[row];
    for (int col = 0; col < M; ++col) {
      A_large_true.block<Dim, Dim>(Dim * row, Dim * col) = A[row][col];
    }
  }
  const auto x_large_true = A_large_true.inverse() * b_large_true;

  timer::tic();
  Eigen::LDLT<Eigen::MatrixXd> eigen_ldlt;
  eigen_ldlt.compute(A_large_true);
  timer::toc(1);

  //
  timer::tic();
  BlockCholeskyDecomposer<double, Dim, M> block_ldlt;
  block_ldlt.DecomposeMatrix(A);
  timer::toc(1);

  const auto x = block_ldlt.SolveLinearEquation(b);

  const auto L_est = block_ldlt.GetMatrixL();
  const auto D_est = block_ldlt.GetMatrixD();

  Eigen::MatrixXd L_mat_est(Dim * M, Dim * M);
  Eigen::MatrixXd D_mat_est(Dim * M, Dim * M);
  L_mat_est.setZero();
  D_mat_est.setZero();
  for (int row = 0; row < M; ++row) {
    D_mat_est.block<Dim, Dim>(Dim * row, Dim * row) = D_est[row];
    for (int col = 0; col <= row; ++col) {
      L_mat_est.block<Dim, Dim>(Dim * row, Dim * col) = L_est[row][col];
    }
  }
  std::cerr << D_mat_est << std::endl;

  auto A_mat_est = L_mat_est * D_mat_est * L_mat_est.transpose();
  std::cerr << "A_large diff:\n" << A_large_true - A_mat_est << std::endl;
  for (int i = 0; i < M; ++i) {
    // std::cerr << (x[i] - x_large_true.block<Dim, 1>(Dim * i, 0)) <<
    // std::endl;
    std::cerr << (x[i] - x_large_true.block<Dim, 1>(Dim * i, 0)).norm()
              << std::endl;
    // std::cerr << x_large_true.block<Dim, 1>(Dim * i, 0) << std::endl;
  }

  return 0;
}
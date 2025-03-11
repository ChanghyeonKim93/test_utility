#include <iostream>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Sparse"
#include "eigen3/Eigen/SparseCholesky"

#define Dimension 2
#define NumBlock 3

int main() {
  /*
    11   0   0  14   0  16
     0  22   0   0  25  26
     0   0  33  34   0  36
    41   0  43  44   0  46
  */
  Eigen::MatrixXd large_A(4, 6);
  Eigen::SparseMatrix<double> sparse_mat;
  sparse_mat.resize(4, 6);  // Change sm1 to a m x n matrix.
  sparse_mat.reserve(100);  // Allocate room for nnz nonzeros elements.

  sparse_mat.insert(0, 0) = 11;
  sparse_mat.insert(0, 3) = 41;
  sparse_mat.insert(1, 1) = 22;
  sparse_mat.insert(2, 2) = 33;
  sparse_mat.insert(3, 2) = 43;
  sparse_mat.insert(0, 3) = 14;
  sparse_mat.insert(2, 3) = 34;
  sparse_mat.insert(3, 3) = 44;
  sparse_mat.insert(1, 4) = 25;
  sparse_mat.insert(0, 5) = 16;
  sparse_mat.insert(1, 5) = 26;
  sparse_mat.insert(2, 5) = 36;
  sparse_mat.insert(3, 5) = 46;
  std::cout << "Number of Rows:\n" << large_A.rows() << std::endl;

  Eigen::VectorXd xe = Eigen::VectorXd::Constant(large_A.rows(), 1);
  std::cout << xe << std::endl;
  Eigen::VectorXd b = large_A * xe;

  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
  Eigen::VectorXd x = solver.compute(sparse_mat).solve(b);
  std::cout << x << std::endl;

  return 0;
}

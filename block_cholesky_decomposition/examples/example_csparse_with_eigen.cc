#include <iostream>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "thirdparty/block_hessian.h"
#include "thirdparty/csparse.h"

#define Dimension 6

int main() {
  // csparse::SparseMatrix sparse_mat;
  BlockHessian<Dimension> block_hessian;

  Eigen::Matrix<double, 6, 6> block_element;
  block_element.setIdentity();
  block_hessian.AddBlockElement(0, 0, block_element);

  block_element.setIdentity();
  block_element(0, 1) = 4;
  block_element(1, 0) = 4;
  block_hessian.AddBlockElement(0, 1, block_element);
  block_hessian.AddBlockElement(1, 0, block_element.transpose());

  block_element.setIdentity();
  block_element(2, 4) = 7;
  block_element(4, 2) = 7;
  block_hessian.AddBlockElement(2, 4, block_element);
  block_hessian.AddBlockElement(4, 2, block_element.transpose());

  return 0;
}

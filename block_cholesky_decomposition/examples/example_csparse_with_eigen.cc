#include <iostream>
#include <vector>

#include "Eigen/Dense"
#include "thirdparty/csparse.h"

void AddElementToSMat(const int row, const int col, const double value,
                      csparse::SparseMatrix* smat);

int main() {
  csparse::SparseMatrix sparse_mat;
  sparse_mat.i;

  return 0;
}

void AddElementToSMat(const int row, const int col, const double value,
                      csparse::SparseMatrix* smat) {
  // smat->i;
}
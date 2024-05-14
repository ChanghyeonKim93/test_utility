#include <iostream>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "thirdparty/block_hessian.h"
#include "thirdparty/csparse.h"

#define Dimension 2
#define NumBlock 3

int main() {
  // csparse::SparseMatrix sparse_mat;
  BlockHessian<Dimension, NumBlock> block_hessian;

  std::vector<std::vector<Eigen::Matrix2d>> blks(2);
  for (auto& blk : blks) blk.resize(3);

  /*
    11   0   0  14   0  16
     0  22   0   0  25  26
     0   0  33  34   0  36
    41   0  43  44   0  46
  */

  blks[0][0] << 11, 0, 0, 22;
  blks[1][0] << 0, 0, 41, 0;
  blks[0][1] << 0, 14, 0, 0;
  blks[1][1] << 33, 34, 43, 44;
  blks[0][2] << 0, 16, 25, 26;
  blks[1][2] << 0, 36, 0, 46;

  block_hessian.AddBlockElement(0, 0, blks[0][0]);
  block_hessian.AddBlockElement(1, 0, blks[1][0]);
  block_hessian.AddBlockElement(0, 1, blks[0][1]);
  block_hessian.AddBlockElement(1, 1, blks[1][1]);
  block_hessian.AddBlockElement(0, 2, blks[0][2]);
  block_hessian.AddBlockElement(1, 2, blks[1][2]);

  block_hessian.ConvertToCSparse();

  return 0;
}

#include <iostream>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Sparse"
#include "eigen3/Eigen/SparseCholesky"
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

  Eigen::MatrixXd large_A(4, 6);
  large_A << 11, 0, 0, 14, 0, 16, 0, 22, 0, 0, 25, 26, 0, 0, 33, 34, 0, 36, 41,
      0, 43, 44, 0, 46;

  Eigen::SimplicialCholesky cholsol;

  block_hessian.AddBlockElement(0, 0, blks[0][0]);
  block_hessian.AddBlockElement(1, 0, blks[1][0]);
  block_hessian.AddBlockElement(0, 1, blks[0][1]);
  block_hessian.AddBlockElement(1, 1, blks[1][1]);
  block_hessian.AddBlockElement(0, 2, blks[0][2]);
  block_hessian.AddBlockElement(1, 2, blks[1][2]);

  block_hessian.ConvertToCSparse();

  csparse::SparseMatrix A = block_hessian.ConvertToCSparse();
  // csparse::Symbolic* S = csparse::cs_schol(&cs, 1);

  // csparse::Numeric* N = csparse::cs_chol(&cs, S);
  csparse::SparseMatrix* AT = csparse::cs_transpose(&A, 1); /* AT = A' */

  std::cerr << "AT: " << AT << std::endl;

  double* x;
  csparse::css* S;
  csparse::csn* N;
  int n, ok;
  n = A.n;
  S = cs_schol(&A, 1); /* ordering and symbolic analysis */
  N = cs_chol(&A, S);  /* numeric Cholesky factorization */
  x = reinterpret_cast<double*>(csparse::cs_malloc(n, sizeof(double)));
  ok = (S && N && x);
  std::cerr << "OK: " << ok << std::endl;
  for (int i = 0; i < 3; ++i)
    std::cerr << "elem : " << *(N->L->x + i) << std::endl;

  // if (ok) {
  //   cs_ipvec(n, S->Pinv, b, x); /* x = P*b */
  //   cs_lsolve(N->L, x);         /* x = L\x */
  //   cs_ltsolve(N->L, x);        /* x = L'\x */
  //   cs_pvec(n, S->Pinv, x, b);  /* b = P'*x */
  // }
  // cs_free(x);
  // cs_sfree(S);
  // cs_nfree(N);

  return 0;
}

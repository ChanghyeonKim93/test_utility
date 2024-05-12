#ifndef THIRDPARTY_BLOCK_CSPARSE_H_
#define THIRDPARTY_BLOCK_CSPARSE_H_

#include <iostream>
#include <map>

#include "thirdparty/csparse.h"

template <int BlockSize, int NumBlockAlongRow, int NumBlockAlongCol>
class BlockHessian {
 public:
  /// @brief Constructor of BlockHessian. Assume the Positive definite and
  /// symmetric matrix. Upper triangular elements are only saved.
  BlockHessian()
      : num_upper_triangle_elements_in_block_(BlockSize * (BlockSize + 1) / 2) {
  }

  /// @brief Destructor of BlockHessian.
  ~BlockHessian() {}

  void AddBlockElement(
      const int block_row, const int block_col,
      const Eigen::Matrix<double, BlockSize, BlockSize>& block_element) {
    // const int start_row = block_row * BlockSize;
    // const int start_col = block_col * BlockSize;

    std::cerr << "block key:" << GetBlockKey(block_row, block_col) << std::endl;
    for (int j = 0; j < BlockSize; ++j) {
      int index = j * BlockSize;
      for (int i = j; i < BlockSize; ++i) {
        block_hessian_[GetBlockKey(block_row, block_col)][index] =
            block_element(i, j);
        ++index;
      }
    }

    for (const auto& [key, block_elem] : block_hessian_) {
      for (int i = 0; i < num_upper_triangle_elements_in_block_; ++i) {
        std::cerr << block_elem[i] << ", ";
      }
      std::cerr << "\n";
    }
  }

  /// @brief Call this function after finalizing the Hessian matrix.
  csparse::SparseMatrix ConvertToCSparse() {
    csparse::SparseMatrix csparse_mat;
    csparse_mat.m = Dimension * NumBlockAlongRow;
    csparse_mat.n = Dimension * NumBlockAlongCol;
    csparse_mat.i = static_cast<int*>(csparse::cs_calloc());

    csparse_mat.nzmax = num_upper_triangle_elements_in_block_ *
                        100;      // TODO(changhyeonkim93): set valid value
    csparse_mat.p csparse_mat.x;  // TODO(changhyeonkim93): set valid value

    return csparse_mat;
  }

 private:
  inline int64_t GetBlockKey(const int block_row, const int block_col) {
    return (static_cast<int64_t>(block_col) << 32 |
            static_cast<int64_t>(block_row));
  }

 private:
  int num_upper_triangle_elements_in_block_;
  std::map<int64_t, double[BlockSize * (BlockSize + 1) / 2]> block_hessian_;
};

#endif  // THIRDPARTY_BLOCK_CSPARSE_H_
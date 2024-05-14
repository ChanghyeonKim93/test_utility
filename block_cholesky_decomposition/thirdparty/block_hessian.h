#ifndef THIRDPARTY_BLOCK_CSPARSE_H_
#define THIRDPARTY_BLOCK_CSPARSE_H_

#include <iostream>
#include <map>

#include "thirdparty/csparse.h"

template <int BlockSize, int NumBlockAlongAxis>
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
    // for (int j = 0; j < BlockSize; ++j) {
    //   int index = j * BlockSize;
    //   for (int i = j; i < BlockSize; ++i) {
    //     block_hessian_[GetBlockKey(block_row, block_col)][index] =
    //         block_element(i, j);
    //     ++index;
    //   }
    // }
    for (int j = 0; j < BlockSize; ++j)
      for (int i = j; i < BlockSize; ++i)
        block_hessian_[GetBlockKey(block_row, block_col)] = block_element;
  }

  /// @brief Call this function after finalizing the Hessian matrix.
  csparse::SparseMatrix ConvertToCSparse() {
    std::vector<std::map<int, double>> row_and_elements;
    row_and_elements.resize(BlockSize * NumBlockAlongAxis);

    int block_row = -1, block_col = -1;
    for (const auto& [key, block_element] : block_hessian_) {
      GetBlockRowAndColumn(key, &block_row, &block_col);
      const int start_col = block_col * BlockSize;
      const int start_row = block_row * BlockSize;
      int num_nonzero_in_column = 0;
      int col = start_col;
      for (int j = 0; j < BlockSize; ++j, ++col) {
        int row = start_row;
        for (int i = 0; i < BlockSize; ++i, ++row) {
          if (std::abs(block_element(i, j)) >
              std::numeric_limits<double>::min())
            row_and_elements[col].insert({row, block_element(i, j)});
        }
      }
    }

    std::cerr << "num column : " << row_and_elements.size() << std::endl;
    for (size_t col = 0; col < row_and_elements.size(); ++col) {
      const auto& elems_in_column = row_and_elements[col];
      for (const auto& [row, elem] : elems_in_column)
        std::cerr << "row and elem in col [ " << col << "]: " << row << " , "
                  << elem << std::endl;
    }

    csparse::SparseMatrix csparse_mat;
    csparse_mat.nzmax = 0;  // total number of nonzero elements
    csparse_mat.m = BlockSize * NumBlockAlongAxis;  // the number of rows
    csparse_mat.n = BlockSize * NumBlockAlongAxis;  // the number of columns
    csparse_mat.i = static_cast<int*>(
        csparse::cs_calloc(csparse_mat.nzmax, sizeof(int)));  // row indices
    csparse_mat.x = static_cast<double*>(csparse::cs_calloc(
        csparse_mat.nzmax, sizeof(double)));  // nonzero elements
    csparse_mat.p;

    csparse_mat.nz = -1;

    return csparse_mat;
  }

 private:
  inline int64_t GetBlockKey(const int block_row, const int block_col) {
    return (static_cast<int64_t>(block_col) << 32 |
            static_cast<int64_t>(block_row));
  }

  inline void GetBlockRowAndColumn(const int64_t block_key, int32_t* row,
                                   int32_t* col) {
    if (row == nullptr || col == nullptr)
      throw std::runtime_error("row or column ptr is nullptr.");

    *row = static_cast<int32_t>(block_key & 0xFFFFFFFF);
    *col = static_cast<int32_t>(block_key >> 32);
  }

 private:
  int num_upper_triangle_elements_in_block_;
  std::map<int64_t, Eigen::Matrix<double, BlockSize, BlockSize>> block_hessian_;
};

#endif  // THIRDPARTY_BLOCK_CSPARSE_H_
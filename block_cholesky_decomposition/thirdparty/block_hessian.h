#ifndef THIRDPARTY_BLOCK_CSPARSE_H_
#define THIRDPARTY_BLOCK_CSPARSE_H_

#include <iostream>
#include <map>

#include "thirdparty/csparse.h"

#define DOUBLE_ZERO 1e-16

template <int BlockSize, int NumBlockAlongAxis>
class BlockHessian {
 public:
  /// @brief Constructor of BlockHessian. Assume the Positive definite and
  /// symmetric matrix. Upper triangular elements are only saved.
  BlockHessian() {}

  /// @brief Destructor of BlockHessian.
  ~BlockHessian() {}

  void Reset() {}

  void AddBlockElement(
      const int block_row, const int block_col,
      const Eigen::Matrix<double, BlockSize, BlockSize>& block_element) {
    block_hessian_[GetBlockKey(block_row, block_col)] = block_element;
  }

  /// @brief Call this function after finalizing the Hessian matrix.
  csparse::SparseMatrix ConvertToCSparse() {
    std::vector<std::map<int, double>> compressed_column_list;
    compressed_column_list.resize(BlockSize * NumBlockAlongAxis);

    int block_row = -1, block_col = -1;
    int num_nonzero = 0;
    for (const auto& [key, block_element] : block_hessian_) {
      GetBlockRowAndColumn(key, &block_row, &block_col);
      const int start_col = block_col * BlockSize;
      const int start_row = block_row * BlockSize;
      int col = start_col;
      for (int j = 0; j < BlockSize; ++j, ++col) {
        int row = start_row;
        for (int i = 0; i < BlockSize; ++i, ++row) {
          if (std::abs(block_element(i, j)) <= DOUBLE_ZERO) continue;
          ++num_nonzero;
          compressed_column_list[col].insert({row, block_element(i, j)});
        }
      }
    }

    for (size_t col = 0; col < compressed_column_list.size(); ++col) {
      const auto& compressed_column = compressed_column_list[col];
      for (const auto& [row, elem] : compressed_column)
        std::cerr << "row , elem in col [ " << col << "]: " << row << " , "
                  << elem << std::endl;
    }

    csparse::SparseMatrix csparse_mat;
    csparse_mat.num_nonzero = num_nonzero;  // total number of nonzero elements
    csparse_mat.m = BlockSize * NumBlockAlongAxis;  // the number of rows
    csparse_mat.n = csparse_mat.m;                  // the number of columns
    csparse_mat.i = static_cast<int*>(csparse::cs_calloc(
        csparse_mat.num_nonzero, sizeof(int)));  // row indices
    csparse_mat.x = static_cast<double*>(csparse::cs_calloc(
        csparse_mat.num_nonzero, sizeof(double)));  // nonzero elements
    csparse_mat.p = static_cast<int*>(
        csparse::cs_calloc(csparse_mat.n + 1, sizeof(int)));  // column indices
    int* row_index_ptr = csparse_mat.i;
    double* element_ptr = csparse_mat.x;
    int* col_index_ptr = csparse_mat.p;
    int num_accum_elems = 0;
    for (size_t col = 0; col < compressed_column_list.size(); ++col) {
      *(col_index_ptr++) = num_accum_elems;
      num_accum_elems += compressed_column_list[col].size();
      for (const auto& [row, elem] : compressed_column_list[col]) {
        *(row_index_ptr++) = row;
        *(element_ptr++) = elem;
      }
    }
    *col_index_ptr = num_nonzero;

    csparse_mat.nz = -1;

    for (int i = 0; i < BlockSize * NumBlockAlongAxis + 1; ++i) {
      std::cerr << "col, start ind: " << i << ", " << *(csparse_mat.p + i)
                << "\n";
    }

    for (int i = 0; i < num_nonzero; ++i) {
      std::cerr << "row, val: " << *(csparse_mat.i + i) << ", "
                << *(csparse_mat.x + i) << "\n";
    }

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
  std::map<int64_t, Eigen::Matrix<double, BlockSize, BlockSize>> block_hessian_;
};

#endif  // THIRDPARTY_BLOCK_CSPARSE_H_
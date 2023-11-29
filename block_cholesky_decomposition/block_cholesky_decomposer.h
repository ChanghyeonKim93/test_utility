#include <type_traits>

#include "eigen3/Eigen/Dense"

/// @brief Cholesky LDLt decomposer for symmetric indefinite block matrix
/// @tparam Numeric Numeric type of the matrix. (only `float` and `double` are
/// supported.)
/// @tparam DimOfBlock Dimension of each block matrix
/// @tparam NumBlocks The number of block matrices along each axis. Total
/// dimension of the operand matrix is {DimOfBlock * NumBlocks} X
/// {DimOfBlock * NumBlocks}
template <typename Numeric, int DimOfBlock, int NumBlocks>
class BlockCholeskyDecomposer {
 private:
  using BlockMatrix = Eigen::Matrix<double, DimOfBlock, DimOfBlock>;
  using BlockVector = Eigen::Matrix<double, DimOfBlock, 1>;

 public:
  /// @brief Constructor. It reserves private storages.
  BlockCholeskyDecomposer() {
    if (!(std::is_same<Numeric, float>::value) &&
        !(std::is_same<Numeric, double>::value)) {
      throw std::runtime_error(
          "Input type is not supported! Only `double` and `float` are "
          "supported.");
    }

    InitializeMatrices();
  }

  /// @brief Destructor
  ~BlockCholeskyDecomposer() {}

  /// @brief Reset the private storage
  void Reset() {
    for (int row = 0; row < NumBlocks; ++row) {
      Lt_inv_[row].setZero();
      for (int col = 0; col < row; ++col) {
        L_[row][col].setZero();
      }
    }

    for (int index = 0; index < NumBlocks; ++index)  //
      D_[index].setZero();
  }

  /// @brief Decompose block full matrix
  /// @param block_full_matrix
  BlockCholeskyDecomposer& DecomposeMatrix(
      const std::vector<std::vector<BlockMatrix>>& block_full_matrix) {
    if (block_full_matrix.size() != NumBlocks ||
        block_full_matrix.front().size() != NumBlocks) {
      throw std::runtime_error(
          "block_full_matrix.size() != NumBlocks "
          "||block_full_matrix.front().size() != NumBlocks");
    }

    Reset();

    const auto& A = block_full_matrix;
    // for (int row = 0; row < NumBlocks; ++row) {
    //   for (int col = 0; col < NumBlocks; ++col) {
    //     std::cerr << A[row][col] << std::endl;
    //   }
    // }
    for (int j = 0; j < NumBlocks; j++) {
      BlockMatrix mat_sum;
      mat_sum.setZero();
      for (int k = 0; k < j; ++k) {
        const auto& Ljk = L_[j][k];
        const auto& Dk = D_[k];
        mat_sum += Ljk * Dk * Ljk.transpose();
      }
      D_[j] = A[j][j] - mat_sum;

      for (int i = j; i < NumBlocks; ++i) {
        mat_sum.setZero();
        for (int k = 0; k < j; ++k) {
          const auto& Lik = L_[i][k];
          const auto& Ljk = L_[j][k];
          const auto& Dk = D_[k];
          mat_sum += Lik * Dk * Ljk.transpose();
        }
        L_[i][j] = (A[i][j] - mat_sum) * D_[j].inverse();
        if (i == j) Lt_inv_[i] = L_[i][i].transpose().inverse();
      }
    }

    return *this;
  }

  /// @brief Solve a block linear equation, A*x = b with respect to x
  /// @param block_vector_in_rhs right hand side vector of A*x = b
  /// @return block vector which is the solution of A*x = b
  std::vector<BlockVector> SolveLinearEquation(
      const std::vector<BlockVector>& block_vector_in_rhs) {
    // Solve A*x = b --> L*D*L'*x = b
    // Step 1)
    //  Define y = D*L'*x, and solve L*y = b w.r.t. y by forward substitution
    // Step 2)
    //  Define z = L'*x, and solve D*z = y w.r.t. z (D is block diagonal.)
    // Step 3)
    //  Solve L'*x = z w.r.t. x by backward substitution

    const auto& b = block_vector_in_rhs;

    // 1) Forward substitution
    // zi = Lii^{-1}*(bi - sum_{j=0}^{i-1} Lij*zj)
    std::vector<BlockVector> z(NumBlocks);
    BlockVector sum_Lz;
    for (int i = 0; i < NumBlocks; ++i) {
      sum_Lz.setZero();
      for (int j = 0; j <= i - 1; ++j) {
        sum_Lz += L_[i][j] * z[j];
      }
      // z[i] = L_[i][i].inverse() * (b[i] - sum_Lz);
      z[i] = Lt_inv_[i].transpose() * (b[i] - sum_Lz);
    }

    // 2) Diagonal inverse
    // yi = Di^{-1} * zi
    auto& y = z;
    // std::vector<BlockVector> y(NumBlocks);
    for (int i = 0; i < NumBlocks; ++i) {
      y[i] = D_[i].inverse() * z[i];
    }

    // 3) Backward substitution
    // xi = {Lii^{\top}}^{-1} * (yi - sum_{j=i+1}^{N-1}(Lji^{\top}*xj) )
    std::vector<BlockVector> x(NumBlocks);
    for (int i = NumBlocks - 1; i >= 0; --i) {
      sum_Lz.setZero();
      for (int j = i + 1; j <= NumBlocks - 1; ++j) {
        sum_Lz += L_[j][i].transpose() * x[j];
      }
      // x[i] = L_[i][i].transpose().inverse() * (y[i] - sum_Lz);
      x[i] = Lt_inv_[i] * (y[i] - sum_Lz);
      // x[i] = ((y[i] - sum_Lz).transpose() * L_[i][i].inverse()).transpose();
    }

    return x;
  }

  /// @brief Get lower block triangle matrix of LDLt
  /// @return lower block triangle matrix of LDLt
  const std::vector<std::vector<BlockMatrix>>& GetMatrixL() const { return L_; }

  /// @brief Get block diagonal matrix of LDLt
  /// @return block diagonal matrix of LDLt
  const std::vector<BlockMatrix>& GetMatrixD() const { return D_; }

 private:
  void InitializeMatrices() {
    Lt_inv_.resize(NumBlocks);
    L_.resize(NumBlocks);
    for (int row = 0; row < NumBlocks; ++row) {
      Lt_inv_[row].setZero();
      L_[row].resize(row + 1);
      for (int col = 0; col < row; ++col)  //
        L_[row][col].setZero();
    }

    D_.resize(NumBlocks);
    for (int index = 0; index < NumBlocks; ++index)  //
      D_[index].setZero();
  }

 private:
  std::vector<std::vector<BlockMatrix>> L_;  // NumBlocks*(NumBlocks+1)/2
  std::vector<BlockMatrix> Lt_inv_;          // NumBlocks*(NumBlocks+1)/2
  std::vector<BlockMatrix> D_;               // NumBlocks
};
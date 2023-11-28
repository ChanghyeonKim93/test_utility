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
    for (int row = 0; row < NumBlocks; ++row)
      for (int col = 0; col < row; ++col)  //
        L_[row][col].setZero();

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
    for (int row = 0; row < NumBlocks; ++row) {
      for (int col = 0; col < NumBlocks; ++col) {
        std::cerr << A[row][col] << std::endl;
      }
    }
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
      }
    }

    return *this;
  }

  std::vector<BlockVector> SolveLinearEquation(
      const std::vector<BlockVector>& block_vector_in_rhs) {
    // Solve A*x = b --> L*D*L'*x = b
    // Step 1)
    //  Define y = D*L'*x, and solve L*y = b w.r.t. y by forward substitution
    // Step 2)
    //  Define z = L'*x, and solve D*z = y w.r.t. z (D is block diagonal.)
    // Step 3)
    //  Solve L'*x = z w.r.t. x by backward substitution

    // TODO(Changhyeon Kim): implement solver.
    const auto& b = block_vector_in_rhs;
    std::vector<BlockVector> x(NumBlocks);
    for (int index = 0; index < NumBlocks; ++index)  //
      x[index].setZero();

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
    L_.resize(NumBlocks);
    for (int row = 0; row < NumBlocks; ++row) {
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
  std::vector<BlockMatrix> D_;               // NumBlocks
};
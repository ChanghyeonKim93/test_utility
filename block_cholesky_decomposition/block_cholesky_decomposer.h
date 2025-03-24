#include <type_traits>

#include "eigen3/Eigen/Dense"
// #include "eigen3/Eigen/StdVector"

/// @brief Cholesky LDLt decomposer for symmetric indefinite block matrix
/// @tparam Numeric Numeric type of the matrix. (only `float` and `double` are
/// supported.)
/// @tparam kDimBlock Dimension of each block matrix
/// @tparam kNumBlock The number of block matrices along each axis. Total
/// dimension of the operand matrix is {kDimBlock * kNumBlock} X
/// {kDimBlock * kNumBlock}
template <typename Numeric, int kDimBlock, int kNumBlock>
class BlockCholeskyDecomposer {
 private:
  using BlockMatrix = Eigen::Matrix<double, kDimBlock, kDimBlock>;
  using BlockVector = Eigen::Matrix<double, kDimBlock, 1>;

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
    for (int row = 0; row < kNumBlock; ++row) {
      for (int col = 0; col <= row; ++col) {
        L_[row][col].setZero();
        Linv_[row][col].setZero();
      }
    }

    for (int row = 0; row < kNumBlock; ++row) {
      D_[row].setZero();
      Dinv_[row].setZero();
    }
  }

  /// @brief Decompose block full matrix
  /// @param block_full_matrix
  BlockCholeskyDecomposer& DecomposeMatrix(
      const std::vector<std::vector<BlockMatrix>>& block_full_matrix) {
    if (block_full_matrix.size() != kNumBlock ||
        block_full_matrix.front().size() != kNumBlock) {
      throw std::runtime_error(
          "block_full_matrix.size() != kNumBlock "
          "||block_full_matrix.front().size() != kNumBlock");
    }

    Reset();

    const auto M = kNumBlock;
    const auto& A = block_full_matrix;

    for (int i = 0; i < M; ++i) L_[i][i].setIdentity();

    for (int i = 0; i < M; ++i) {
      for (int j = 0; j < i; ++j) {
        BlockMatrix sum_Lik_Dk_Ljkt;
        sum_Lik_Dk_Ljkt.setZero();
        for (int k = 0; k < j; ++k) {
          const auto& Lik = L_[i][k];
          const auto& Ljk = L_[j][k];
          const auto& Dk = D_[k];
          sum_Lik_Dk_Ljkt.noalias() += Lik * Dk * Ljk.transpose();
        }
        L_[i][j] = (A[i][j] - sum_Lik_Dk_Ljkt) * Dinv_[j];
      }

      BlockMatrix sum_Lik_Dk_Likt;
      sum_Lik_Dk_Likt.setZero();
      for (int k = 0; k < i; ++k) {
        const auto& Lik = L_[i][k];
        const auto& Dk = D_[k];
        sum_Lik_Dk_Likt.noalias() += Lik * Dk * Lik.transpose();
      }
      D_[i] = A[i][i] - sum_Lik_Dk_Likt;
      Dinv_[i] = D_[i].inverse();
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
    //  Define y = D*L'*x, and solve L*y = b w.r.t. y by forward substitution
    const auto M = kNumBlock;
    std::vector<BlockVector> y(M, BlockVector::Zero());
    BlockVector sum_Ly(BlockVector::Zero());
    for (int i = 0; i < M; ++i) {
      Linv_[i][i] = L_[i][i].inverse();
      sum_Ly.setZero();
      for (int j = 0; j < i; ++j) sum_Ly += L_[i][j] * y[j];
      y[i] = Linv_[i][i] * (b[i] - sum_Ly);
    }

    // 2) Diagonal inverse
    //  Define z = L'*x, and solve D*z = y w.r.t. z (D is block diagonal.)
    auto& z = y;
    for (int i = 0; i < M; ++i) {
      z[i] = Dinv_[i] * y[i];
    }

    // 3) Backward substitution
    //  Solve L'*x = z w.r.t. x by backward substitution
    BlockVector sum_Lz(BlockVector::Zero());
    auto& x = z;
    for (int i = M - 1; i >= 0; --i) {
      sum_Lz.setZero();
      for (int j = M - 1; j > i; --j) {
        sum_Lz += L_[j][i].transpose() * x[j];
      }
      x[i] = L_[i][i].transpose().inverse() * (z[i] - sum_Lz);
    }

    // std::vector<BlockVector> z(M, BlockVector::Zero());
    // BlockVector sum_Lz(BlockVector::Zero());
    // for (int i = 0; i < M; ++i) {
    //   sum_Lz.setZero();
    //   for (int j = 0; j <= i - 1; ++j) {
    //     sum_Lz += L_[i][j] * z[j];
    //   }
    //   z[i] = b[i] - sum_Lz;
    // }

    // // 2) Diagonal inverse
    // // yi = Di^{-1} * zi
    // auto& y = z;
    // for (int i = 0; i < M; ++i) {
    //   y[i] = D_[i].inverse() * z[i];
    // }

    // // 3) Backward substitution
    // // xi = {Lii^{\top}}^{-1} * (yi - sum_{j=i+1}^{N-1}(Lji^{\top}*xj) )
    // std::vector<BlockVector> x(M);
    // for (int i = M - 1; i >= 0; --i) {
    //   sum_Lz.setZero();
    //   for (int j = i + 1; j <= M - 1; ++j) {
    //     sum_Lz += L_[j][i].transpose() * x[j];
    //   }
    //   x[i] = y[i] - sum_Lz;
    // }

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
    L_.resize(kNumBlock);
    Linv_.resize(kNumBlock);
    for (int row = 0; row < kNumBlock; ++row) {
      L_.at(row).resize(row + 1);
      Linv_.at(row).resize(row + 1);
      for (int col = 0; col <= row; ++col) {
        L_.at(row).at(col).setZero();
        Linv_.at(row).at(col).setZero();
      }
    }

    D_.resize(kNumBlock);
    Dinv_.resize(kNumBlock);
    for (int row = 0; row < kNumBlock; ++row) {
      Dinv_[row].setZero();
      D_[row].setZero();
    }
  }

 private:
  std::vector<std::vector<BlockMatrix>> L_;     // kNumBlock*(kNumBlock+1)/2
  std::vector<std::vector<BlockMatrix>> Linv_;  // kNumBlock*(kNumBlock+1)/2
  std::vector<BlockMatrix> D_;                  // kNumBlock
  std::vector<BlockMatrix> Dinv_;               // kNumBlock
};
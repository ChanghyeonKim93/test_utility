#ifndef _CS_H
#define _CS_H

// Reference: https://people.math.sc.edu/Burkardt/c_src/csparse/csparse.html
/*
 CS_CHOL : sparse Cholesky
 CS_CHOLSOL : x=A\b using sparse Cholesky
*/
#define CS_VER 1 /* CSparse Version 1.2.0 */
#define CS_SUBVER 2
#define CS_SUBSUB 0
#define CS_DATE "Mar 6, 2006" /* CSparse release date */
#define CS_COPYRIGHT "Copyright (c) Timothy A. Davis, 2006"

namespace csparse {

/* --- primary CSparse routines and data structures ------------------------- */
struct SparseMatrix /* matrix in compressed-column or triplet form */
{
  int nzmax; /* maximum number of entries */
  int m;     /* number of rows */
  int n;     /* number of columns */
  int* p;    /* column pointers (size n+1) or col indices (size nzmax) */
  int* i;    /* row indices, size nzmax */
  double* x; /* numerical values, size nzmax */
  int nz;    /* # of entries in triplet matrix, -1 for compressed-col */
};

SparseMatrix* cs_add(const SparseMatrix* A, const SparseMatrix* B, double alpha,
                     double beta);
int cs_cholsol(const SparseMatrix* A, double* b, int order);
int cs_dupl(SparseMatrix* A);
int cs_entry(SparseMatrix* T, int i, int j, double x);
int cs_lusol(const SparseMatrix* A, double* b, int order, double tol);
int cs_gaxpy(const SparseMatrix* A, const double* x, double* y);
SparseMatrix* cs_multiply(const SparseMatrix* A, const SparseMatrix* B);
int cs_qrsol(const SparseMatrix* A, double* b, int order);
SparseMatrix* cs_transpose(const SparseMatrix* A, int values);
SparseMatrix* cs_triplet(const SparseMatrix* T);
double cs_norm(const SparseMatrix* A);
int cs_print(const SparseMatrix* A, int brief);
SparseMatrix* cs_load(FILE* f);
/* utilities */
void* cs_calloc(int n, size_t size);
void* cs_free(void* p);
void* cs_realloc(void* p, int n, size_t size, int* ok);
SparseMatrix* cs_spalloc(int m, int n, int nzmax, int values, int triplet);
SparseMatrix* cs_spfree(SparseMatrix* A);
int cs_sprealloc(SparseMatrix* A, int nzmax);
void* cs_malloc(int n, size_t size);

/* --- secondary CSparse routines and data structures ----------------------- */
struct Symbolic /* symbolic Cholesky, LU, or QR analysis */
{
  int* Pinv;   /* inverse row perm. for QR, fill red. perm for Chol */
  int* Q;      /* fill-reducing column permutation for LU and QR */
  int* parent; /* elimination tree for Cholesky and QR */
  int* cp;     /* column pointers for Cholesky, row counts for QR */
  int m2;      /* # of rows for QR, after adding fictitious rows */
  int lnz;     /* # entries in L for LU or Cholesky; in V for QR */
  int unz;     /* # entries in U for LU; in R for QR */
};

struct Numeric /* numeric Cholesky, LU, or QR factorization */
{
  SparseMatrix* L; /* L for LU and Cholesky, V for QR */
  SparseMatrix* U; /* U for LU, R for QR, not used for Cholesky */
  int* Pinv;       /* partial pivoting for LU */
  double* B;       /* beta [0..n-1] for QR */
};

struct DmpermResults /* cs_dmperm or cs_scc output */
{
  int* P;    /* size m, row permutation */
  int* Q;    /* size n, column permutation */
  int* R;    /* size nb+1, block k is rows R[k] to R[k+1]-1 in A(P,Q) */
  int* S;    /* size nb+1, block k is cols S[k] to S[k+1]-1 in A(P,Q) */
  int nb;    /* # of blocks in fine dmperm decomposition */
  int rr[5]; /* coarse row decomposition */
  int cc[5]; /* coarse column decomposition */
};

int* cs_amd(const SparseMatrix* A, int order);
Numeric* cs_chol(const SparseMatrix* A, const Symbolic* S);
DmpermResults* cs_dmperm(const SparseMatrix* A);
int cs_droptol(SparseMatrix* A, double tol);
int cs_dropzeros(SparseMatrix* A);
int cs_happly(const SparseMatrix* V, int i, double beta, double* x);
int cs_ipvec(int n, const int* P, const double* b, double* x);
int cs_lsolve(const SparseMatrix* L, double* x);
int cs_ltsolve(const SparseMatrix* L, double* x);
Numeric* cs_lu(const SparseMatrix* A, const Symbolic* S, double tol);
SparseMatrix* cs_permute(const SparseMatrix* A, const int* P, const int* Q,
                         int values);
int* cs_pinv(const int* P, int n);
int cs_pvec(int n, const int* P, const double* b, double* x);
Numeric* cs_qr(const SparseMatrix* A, const Symbolic* S);
Symbolic* cs_schol(const SparseMatrix* A, int order);
Symbolic* cs_sqr(const SparseMatrix* A, int order, int qr);
SparseMatrix* cs_symperm(const SparseMatrix* A, const int* Pinv, int values);
int cs_usolve(const SparseMatrix* U, double* x);
int cs_utsolve(const SparseMatrix* U, double* x);
int cs_updown(SparseMatrix* L, int sigma, const SparseMatrix* C,
              const int* parent);
/* utilities */
Symbolic* cs_sfree(Symbolic* S);
Numeric* cs_nfree(Numeric* N);
DmpermResults* cs_dfree(DmpermResults* D);

/* --- tertiary CSparse routines -------------------------------------------- */
int* cs_counts(const SparseMatrix* A, const int* parent, const int* post,
               int ata);
int cs_cumsum(int* p, int* c, int n);
int cs_dfs(int j, SparseMatrix* L, int top, int* xi, int* pstack,
           const int* Pinv);
int* cs_etree(const SparseMatrix* A, int ata);
int cs_fkeep(SparseMatrix* A, int (*fkeep)(int, int, double, void*),
             void* other);
double cs_house(double* x, double* beta, int n);
int* cs_maxtrans(const SparseMatrix* A);
int* cs_post(int n, const int* parent);
int cs_reach(SparseMatrix* L, const SparseMatrix* B, int k, int* xi,
             const int* Pinv);
DmpermResults* cs_scc(SparseMatrix* A);
int cs_scatter(const SparseMatrix* A, int j, double beta, int* w, double* x,
               int mark, SparseMatrix* C, int nz);
int cs_splsolve(SparseMatrix* L, const SparseMatrix* B, int k, int* xi,
                double* x, const int* Pinv);
int cs_tdfs(int j, int k, int* head, const int* next, int* post, int* stack);
/* utilities */
DmpermResults* cs_dalloc(int m, int n);
SparseMatrix* cs_done(SparseMatrix* C, void* w, void* x, int ok);
int* cs_idone(int* p, SparseMatrix* C, void* w, int ok);
Numeric* cs_ndone(Numeric* N, SparseMatrix* C, void* w, void* x, int ok);
DmpermResults* cs_ddone(DmpermResults* D, SparseMatrix* C, void* w, int ok);

#define CS_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define CS_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define CS_FLIP(i) (-(i) - 2)
#define CS_UNFLIP(i) (((i) < 0) ? CS_FLIP(i) : (i))
#define CS_MARKED(Ap, j) (Ap[j] < 0)
#define CS_MARK(Ap, j) \
  { Ap[j] = CS_FLIP(Ap[j]); }
#define CS_OVERFLOW(n, size) (n > INT_MAX / (int)size)

}  // namespace csparse

#endif

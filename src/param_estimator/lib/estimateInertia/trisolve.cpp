//
// File: trisolve.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 01-Mar-2024 18:43:15
//

// Include Files
#include "trisolve.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : const double A[9]
//                double B[18]
// Return Type  : void
//
namespace coder {
namespace internal {
void b_trisolve(const double A[9], double B[18])
{
  for (int j{0}; j < 6; j++) {
    int jBcol;
    jBcol = 3 * j;
    for (int k{2}; k >= 0; k--) {
      double d;
      int i;
      int kAcol;
      kAcol = 3 * k;
      i = k + jBcol;
      d = B[i];
      if (d != 0.0) {
        B[i] = d / A[k + kAcol];
        for (int b_i{0}; b_i < k; b_i++) {
          int i1;
          i1 = b_i + jBcol;
          B[i1] -= B[i] * A[b_i + kAcol];
        }
      }
    }
  }
}

//
// Arguments    : const double A[9]
//                double B[18]
// Return Type  : void
//
void trisolve(const double A[9], double B[18])
{
  for (int j{0}; j < 6; j++) {
    int jBcol;
    jBcol = 3 * j - 1;
    for (int k{0}; k < 3; k++) {
      double d;
      int i;
      int kAcol;
      kAcol = 3 * k - 1;
      i = (k + jBcol) + 1;
      d = B[i];
      if (d != 0.0) {
        int i1;
        B[i] = d / A[(k + kAcol) + 1];
        i1 = k + 2;
        for (int b_i{i1}; b_i < 4; b_i++) {
          int i2;
          i2 = b_i + jBcol;
          B[i2] -= B[i] * A[b_i + kAcol];
        }
      }
    }
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for trisolve.cpp
//
// [EOF]
//

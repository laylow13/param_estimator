//
// File: EKFCorrectorAdditive.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 01-Mar-2024 18:43:15
//

// Include Files
#include "EKFCorrectorAdditive.h"
#include "estimateInertia_rtwutil.h"
#include "rt_nonfinite.h"
#include "trisolve.h"
#include "xnrm2.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : double x[6]
//                double S[36]
//                const double residue[3]
//                const double Pxy[18]
//                const double Sy[9]
//                const double H[18]
//                const double Rsqrt[9]
// Return Type  : void
//
namespace coder {
namespace matlabshared {
namespace tracking {
namespace internal {
void EKFCorrectorAdditive::correctStateAndSqrtCovariance(
    double x[6], double S[36], const double residue[3], const double Pxy[18],
    const double Sy[9], const double H[18], const double Rsqrt[9])
{
  double M[54];
  double A[36];
  double b_C[36];
  double C[18];
  double K[18];
  double b_Pxy[18];
  double b_Sy[9];
  double tau[6];
  double work[6];
  double beta1;
  double d;
  double s;
  int aoffset;
  int coffset;
  int i;
  int i1;
  int k;
  for (i = 0; i < 6; i++) {
    b_Pxy[3 * i] = Pxy[i];
    b_Pxy[3 * i + 1] = Pxy[i + 6];
    b_Pxy[3 * i + 2] = Pxy[i + 12];
  }
  std::copy(&b_Pxy[0], &b_Pxy[18], &K[0]);
  for (int j{0}; j < 6; j++) {
    b_Pxy[3 * j] = K[3 * j];
    coffset = 3 * j + 1;
    b_Pxy[coffset] = K[coffset];
    coffset = 3 * j + 2;
    b_Pxy[coffset] = K[coffset];
  }
  ::coder::internal::trisolve(Sy, b_Pxy);
  for (int j{0}; j < 6; j++) {
    C[3 * j] = b_Pxy[3 * j];
    coffset = 3 * j + 1;
    C[coffset] = b_Pxy[coffset];
    coffset = 3 * j + 2;
    C[coffset] = b_Pxy[coffset];
  }
  for (i = 0; i < 3; i++) {
    b_Sy[3 * i] = Sy[i];
    b_Sy[3 * i + 1] = Sy[i + 3];
    b_Sy[3 * i + 2] = Sy[i + 6];
  }
  ::coder::internal::b_trisolve(b_Sy, C);
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      K[i1 + 6 * i] = C[i + 3 * i1];
    }
  }
  d = residue[0];
  s = residue[1];
  beta1 = residue[2];
  for (i = 0; i < 6; i++) {
    x[i] += (K[i] * d + K[i + 6] * s) + K[i + 12] * beta1;
  }
  for (i = 0; i < 18; i++) {
    b_Pxy[i] = -K[i];
  }
  for (i = 0; i < 6; i++) {
    d = b_Pxy[i];
    s = b_Pxy[i + 6];
    beta1 = b_Pxy[i + 12];
    for (i1 = 0; i1 < 6; i1++) {
      A[i + 6 * i1] =
          (d * H[3 * i1] + s * H[3 * i1 + 1]) + beta1 * H[3 * i1 + 2];
    }
  }
  for (int b_i{0}; b_i < 6; b_i++) {
    coffset = b_i + 6 * b_i;
    A[coffset]++;
  }
  for (int j{0}; j < 6; j++) {
    coffset = j * 6;
    for (int b_i{0}; b_i < 6; b_i++) {
      aoffset = b_i * 6;
      s = 0.0;
      for (k = 0; k < 6; k++) {
        s += S[aoffset + k] * A[k * 6 + j];
      }
      b_C[coffset + b_i] = s;
    }
  }
  for (i = 0; i < 3; i++) {
    d = Rsqrt[3 * i];
    s = Rsqrt[3 * i + 1];
    beta1 = Rsqrt[3 * i + 2];
    for (i1 = 0; i1 < 6; i1++) {
      b_Pxy[i + 3 * i1] = (K[i1] * d + K[i1 + 6] * s) + K[i1 + 12] * beta1;
    }
  }
  for (int b_i{0}; b_i < 6; b_i++) {
    for (i = 0; i < 6; i++) {
      M[i + 9 * b_i] = b_C[i + 6 * b_i];
    }
    M[9 * b_i + 6] = b_Pxy[3 * b_i];
    M[9 * b_i + 7] = b_Pxy[3 * b_i + 1];
    M[9 * b_i + 8] = b_Pxy[3 * b_i + 2];
    tau[b_i] = 0.0;
    work[b_i] = 0.0;
  }
  for (int b_i{0}; b_i < 6; b_i++) {
    double atmp;
    int ii;
    ii = b_i * 9 + b_i;
    atmp = M[ii];
    coffset = ii + 2;
    tau[b_i] = 0.0;
    s = ::coder::internal::blas::c_xnrm2(8 - b_i, M, ii + 2);
    if (s != 0.0) {
      d = M[ii];
      beta1 = rt_hypotd_snf(d, s);
      if (d >= 0.0) {
        beta1 = -beta1;
      }
      if (std::abs(beta1) < 1.0020841800044864E-292) {
        aoffset = 0;
        i = (ii - b_i) + 9;
        do {
          aoffset++;
          for (k = coffset; k <= i; k++) {
            M[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (aoffset < 20));
        beta1 = rt_hypotd_snf(
            atmp, ::coder::internal::blas::c_xnrm2(8 - b_i, M, ii + 2));
        if (atmp >= 0.0) {
          beta1 = -beta1;
        }
        tau[b_i] = (beta1 - atmp) / beta1;
        s = 1.0 / (atmp - beta1);
        for (k = coffset; k <= i; k++) {
          M[k - 1] *= s;
        }
        for (k = 0; k < aoffset; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        atmp = beta1;
      } else {
        tau[b_i] = (beta1 - d) / beta1;
        s = 1.0 / (d - beta1);
        i = (ii - b_i) + 9;
        for (k = coffset; k <= i; k++) {
          M[k - 1] *= s;
        }
        atmp = beta1;
      }
    }
    M[ii] = atmp;
    if (b_i + 1 < 6) {
      int lastc;
      int lastv;
      M[ii] = 1.0;
      if (tau[b_i] != 0.0) {
        boolean_T exitg2;
        lastv = 9 - b_i;
        coffset = (ii - b_i) + 8;
        while ((lastv > 0) && (M[coffset] == 0.0)) {
          lastv--;
          coffset--;
        }
        lastc = 4 - b_i;
        exitg2 = false;
        while ((!exitg2) && (lastc + 1 > 0)) {
          int exitg1;
          coffset = (ii + lastc * 9) + 9;
          k = coffset;
          do {
            exitg1 = 0;
            if (k + 1 <= coffset + lastv) {
              if (M[k] != 0.0) {
                exitg1 = 1;
              } else {
                k++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = -1;
      }
      if (lastv > 0) {
        aoffset = ii + 10;
        if (lastc + 1 != 0) {
          std::memset(&work[0], 0,
                      static_cast<unsigned int>(lastc + 1) * sizeof(double));
          i = (ii + 9 * lastc) + 10;
          for (int j{aoffset}; j <= i; j += 9) {
            s = 0.0;
            i1 = (j + lastv) - 1;
            for (k = j; k <= i1; k++) {
              s += M[k - 1] * M[(ii + k) - j];
            }
            coffset = div_nde_s32_floor((j - ii) - 10, 9);
            work[coffset] += s;
          }
        }
        if (!(-tau[b_i] == 0.0)) {
          coffset = ii;
          for (int j{0}; j <= lastc; j++) {
            d = work[j];
            if (d != 0.0) {
              s = d * -tau[b_i];
              i = coffset + 10;
              i1 = lastv + coffset;
              for (aoffset = i; aoffset <= i1 + 9; aoffset++) {
                M[aoffset - 1] += M[((ii + aoffset) - coffset) - 10] * s;
              }
            }
            coffset += 9;
          }
        }
      }
      M[ii] = atmp;
    }
  }
  for (int j{0}; j < 6; j++) {
    for (int b_i{0}; b_i <= j; b_i++) {
      A[b_i + 6 * j] = M[b_i + 9 * j];
    }
    i = j + 2;
    if (i <= 6) {
      std::memset(&A[(j * 6 + i) + -1], 0,
                  static_cast<unsigned int>(-i + 7) * sizeof(double));
    }
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      S[i1 + 6 * i] = A[i + 6 * i1];
    }
  }
}

} // namespace internal
} // namespace tracking
} // namespace matlabshared
} // namespace coder

//
// File trailer for EKFCorrectorAdditive.cpp
//
// [EOF]
//

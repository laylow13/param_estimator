//
// File: trackingEKF.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 01-Mar-2024 18:43:15
//

// Include Files
#include "trackingEKF.h"
#include "InertiaEstimatorEKF.h"
#include "estimateInertia_data.h"
#include "estimateInertia_rtwutil.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double varargin_1[4]
// Return Type  : void
//
namespace coder {
void trackingEKF::predict(const double varargin_1[4])
{
  double M[72];
  double Qsqrt[36];
  double dFdx[36];
  double dv1[36];
  double dv[6];
  double dv2[6];
  double imvec[6];
  double z[6];
  double d;
  double epsilon;
  int aoffset;
  int i;
  int i1;
  int k;
  if (!IsLastJacobianInitialized) {
    IsLastJacobianInitialized = true;
  }
  if (!pIsDistributionsSetup) {
    pIsDistributionsSetup = true;
  }
  if ((!pIsSetStateCovariance) || (pSqrtStateCovarianceScalar != -1.0)) {
    epsilon = pSqrtStateCovarianceScalar;
    for (i = 0; i < 36; i++) {
      pSqrtStateCovariance[i] = epsilon * static_cast<double>(iv[i]);
    }
    pIsSetStateCovariance = true;
    pSqrtStateCovarianceScalar = -1.0;
  }
  if ((!pIsSetProcessNoise) || (pSqrtProcessNoiseScalar != -1.0)) {
    epsilon = pSqrtProcessNoiseScalar;
    for (i = 0; i < 36; i++) {
      pSqrtProcessNoise[i] = epsilon * static_cast<double>(iv[i]);
    }
    pIsSetProcessNoise = true;
    pSqrtProcessNoiseScalar = -1.0;
  }
  if (pIsFirstCallPredict) {
    if (!pIsValidStateTransitionFcn) {
      pIsValidStateTransitionFcn = true;
    }
    pIsFirstCallPredict = false;
  }
  for (i = 0; i < 36; i++) {
    Qsqrt[i] = pSqrtProcessNoise[i];
  }
  for (int b_i{0}; b_i < 6; b_i++) {
    dv[b_i] = pState[b_i];
  }
  for (i = 0; i < 36; i++) {
    dv1[i] = pSqrtStateCovariance[i];
  }
  stateTransitionFcn(dv, varargin_1, z);
  for (int j{0}; j < 6; j++) {
    for (int b_i{0}; b_i < 6; b_i++) {
      imvec[b_i] = dv[b_i];
    }
    d = dv[j];
    epsilon =
        std::fmax(1.4901161193847656E-8, 1.4901161193847656E-8 * std::abs(d));
    imvec[j] = d + epsilon;
    stateTransitionFcn(imvec, varargin_1, dv2);
    for (i = 0; i < 6; i++) {
      dFdx[i + 6 * j] = (dv2[i] - z[i]) / epsilon;
    }
  }
  for (i = 0; i < 6; i++) {
    dv2[i] = dv[i];
  }
  stateTransitionFcn(dv2, varargin_1, dv);
  for (int j{0}; j < 6; j++) {
    for (int b_i{0}; b_i < 6; b_i++) {
      aoffset = b_i * 6;
      epsilon = 0.0;
      for (k = 0; k < 6; k++) {
        epsilon += dv1[aoffset + k] * dFdx[k * 6 + j];
      }
      aoffset = b_i + 12 * j;
      M[aoffset] = epsilon;
      M[aoffset + 6] = Qsqrt[j + 6 * b_i];
    }
    z[j] = 0.0;
    imvec[j] = 0.0;
  }
  for (int b_i{0}; b_i < 6; b_i++) {
    double atmp;
    int ii;
    int knt;
    ii = b_i * 12 + b_i;
    atmp = M[ii];
    aoffset = ii + 2;
    z[b_i] = 0.0;
    epsilon = internal::blas::xnrm2(11 - b_i, M, ii + 2);
    if (epsilon != 0.0) {
      double beta1;
      d = M[ii];
      beta1 = rt_hypotd_snf(d, epsilon);
      if (d >= 0.0) {
        beta1 = -beta1;
      }
      if (std::abs(beta1) < 1.0020841800044864E-292) {
        knt = 0;
        i = (ii - b_i) + 12;
        do {
          knt++;
          for (k = aoffset; k <= i; k++) {
            M[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));
        beta1 = rt_hypotd_snf(atmp, internal::blas::xnrm2(11 - b_i, M, ii + 2));
        if (atmp >= 0.0) {
          beta1 = -beta1;
        }
        z[b_i] = (beta1 - atmp) / beta1;
        epsilon = 1.0 / (atmp - beta1);
        for (k = aoffset; k <= i; k++) {
          M[k - 1] *= epsilon;
        }
        for (k = 0; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        atmp = beta1;
      } else {
        z[b_i] = (beta1 - d) / beta1;
        epsilon = 1.0 / (d - beta1);
        i = (ii - b_i) + 12;
        for (k = aoffset; k <= i; k++) {
          M[k - 1] *= epsilon;
        }
        atmp = beta1;
      }
    }
    M[ii] = atmp;
    if (b_i + 1 < 6) {
      int lastc;
      int lastv;
      M[ii] = 1.0;
      if (z[b_i] != 0.0) {
        boolean_T exitg2;
        lastv = 12 - b_i;
        aoffset = (ii - b_i) + 11;
        while ((lastv > 0) && (M[aoffset] == 0.0)) {
          lastv--;
          aoffset--;
        }
        lastc = 4 - b_i;
        exitg2 = false;
        while ((!exitg2) && (lastc + 1 > 0)) {
          int exitg1;
          aoffset = (ii + lastc * 12) + 12;
          k = aoffset;
          do {
            exitg1 = 0;
            if (k + 1 <= aoffset + lastv) {
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
        knt = ii + 13;
        if (lastc + 1 != 0) {
          std::memset(&imvec[0], 0,
                      static_cast<unsigned int>(lastc + 1) * sizeof(double));
          i = (ii + 12 * lastc) + 13;
          for (int j{knt}; j <= i; j += 12) {
            epsilon = 0.0;
            i1 = (j + lastv) - 1;
            for (k = j; k <= i1; k++) {
              epsilon += M[k - 1] * M[(ii + k) - j];
            }
            aoffset = div_nde_s32_floor((j - ii) - 13, 12);
            imvec[aoffset] += epsilon;
          }
        }
        if (!(-z[b_i] == 0.0)) {
          aoffset = ii;
          for (int j{0}; j <= lastc; j++) {
            d = imvec[j];
            if (d != 0.0) {
              epsilon = d * -z[b_i];
              i = aoffset + 13;
              i1 = lastv + aoffset;
              for (knt = i; knt <= i1 + 12; knt++) {
                M[knt - 1] += M[((ii + knt) - aoffset) - 13] * epsilon;
              }
            }
            aoffset += 12;
          }
        }
      }
      M[ii] = atmp;
    }
  }
  for (int j{0}; j < 6; j++) {
    for (int b_i{0}; b_i <= j; b_i++) {
      Qsqrt[b_i + 6 * j] = M[b_i + 12 * j];
    }
    i = j + 2;
    if (i <= 6) {
      std::memset(&Qsqrt[(j * 6 + i) + -1], 0,
                  static_cast<unsigned int>(-i + 7) * sizeof(double));
    }
  }
  for (int b_i{0}; b_i < 6; b_i++) {
    pState[b_i] = dv[b_i];
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      pSqrtStateCovariance[i1 + 6 * i] = Qsqrt[i + 6 * i1];
    }
  }
  pIsSetStateCovariance = true;
  pSqrtStateCovarianceScalar = -1.0;
  for (i = 0; i < 36; i++) {
    pJacobian[i] = dFdx[i];
  }
}

//
// Arguments    : void
// Return Type  : trackingEKF
//
trackingEKF::trackingEKF() = default;

//
// Arguments    : void
// Return Type  : void
//
trackingEKF::~trackingEKF() = default;

} // namespace coder

//
// File trailer for trackingEKF.cpp
//
// [EOF]
//

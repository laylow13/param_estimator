//
// File: InertiaEstimatorEKF.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 01-Mar-2024 18:43:15
//

// Include Files
#include "InertiaEstimatorEKF.h"
#include "EKFCorrectorAdditive.h"
#include "estimateInertia_data.h"
#include "estimateInertia_rtwutil.h"
#include "estimateInertia_types.h"
#include "rt_nonfinite.h"
#include "trackingEKF.h"
#include "xnrm2.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Declarations
static void estimateInertia_init(InertiaEstimatorEKF *aInstancePtr);

// Function Definitions
//
// Arguments    : void
// Return Type  : estimateInertiaStackData *
//
estimateInertiaStackData *InertiaEstimatorEKF::getStackData()
{
  return &SD_;
}

//
// Arguments    : InertiaEstimatorEKF *aInstancePtr
// Return Type  : void
//
static void estimateInertia_init(InertiaEstimatorEKF *aInstancePtr)
{
  estimateInertiaStackData *localSD;
  localSD = aInstancePtr->getStackData();
  localSD->pd->ekf_not_empty = false;
}

//
// Arguments    : void
// Return Type  : void
//
InertiaEstimatorEKF::InertiaEstimatorEKF()
{
  SD_.pd = &pd_;
  estimateInertia_init(this);
}

//
// Arguments    : void
// Return Type  : void
//
InertiaEstimatorEKF::~InertiaEstimatorEKF() = default;

//
// Arguments    : const double M[3]
//                double sample_t
//                const double Omega[3]
//                double out[42]
// Return Type  : void
//
void InertiaEstimatorEKF::estimateInertia(const double M[3], double sample_t,
                                          const double Omega[3], double out[42])
{
  static const double b_value[36]{1.000499875062461,
                                  0.00099950037468777317,
                                  0.00099950037468777317,
                                  0.00099950037468777317,
                                  0.00099950037468777317,
                                  0.00099950037468777317,
                                  0.0,
                                  1.0004993758114,
                                  0.00099850237106926133,
                                  0.00099850237106926133,
                                  0.00099850237106926133,
                                  0.00099850237106926133,
                                  0.0,
                                  0.0,
                                  1.0004988775565997,
                                  0.00099750635848115622,
                                  0.00099750635848115622,
                                  0.00099750635848115622,
                                  0.0,
                                  0.0,
                                  0.0,
                                  1.0004983802950811,
                                  0.00099651233097119616,
                                  0.00099651233097119616,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  1.000497884023877,
                                  0.00099552028261082272,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  1.0004973887400315};
  static const double c_value[36]{
      0.01, 0.0, 0.0,  0.0, 0.0,  0.0, 0.0, 0.01, 0.0, 0.0,  0.0, 0.0,
      0.0,  0.0, 0.01, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.01, 0.0, 0.0,
      0.0,  0.0, 0.0,  0.0, 0.01, 0.0, 0.0, 0.0,  0.0, 0.0,  0.0, 0.01};
  static const double b_b[18]{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static const double d_value[9]{0.01, 0.0, 0.0, 0.0, 0.01,
                                 0.0,  0.0, 0.0, 0.01};
  static const signed char b_iv[18]{1, 0, 0, 0, 1, 0, 0, 0, 1,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const signed char iv1[18]{1, 0, 0, 0, 0, 0, 0, 1, 0,
                                   0, 0, 0, 0, 0, 1, 0, 0, 0};
  static const signed char b[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  double dv[36];
  double dv1[36];
  double c_M[27];
  double y[18];
  double R[9];
  double Rsqrt[9];
  double b_R[9];
  double stateEstimate[6];
  double b_M[4];
  double tau[3];
  double work[3];
  double d;
  double s;
  int aoffset;
  int b_i;
  int i1;
  int k;
  if (!pd_.ekf_not_empty) {
    //  初始化惯性矩阵估计
    //  初始状态和协方差
    //  创建trackingEKF对象
    pd_.ekf.pIsFirstCallPredict = true;
    pd_.ekf.pIsFirstCallCorrect = true;
    for (int i{0}; i < 6; i++) {
      pd_.ekf.pState[i] = 0.001;
    }
    pd_.ekf.pIsSetStateCovariance = true;
    pd_.ekf.pSqrtStateCovarianceScalar = -1.0;
    pd_.ekf.pIsValidMeasurementFcn = false;
    pd_.ekf.pIsValidStateTransitionFcn = false;
    std::copy(&b_value[0], &b_value[36], &pd_.ekf.pSqrtStateCovariance[0]);
    std::copy(&c_value[0], &c_value[36], &pd_.ekf.pSqrtProcessNoise[0]);
    pd_.ekf.pIsSetProcessNoise = true;
    pd_.ekf.pSqrtProcessNoiseScalar = -1.0;
    std::copy(&d_value[0], &d_value[9], &pd_.ekf.pSqrtMeasurementNoise[0]);
    pd_.ekf.pIsSetMeasurementNoise = true;
    pd_.ekf.pSqrtMeasurementNoiseScalar = -1.0;
    pd_.ekf.IsLastJacobianInitialized = false;
    pd_.ekf.pIsDistributionsSetup = false;
    pd_.ekf.pIsInitialized = false;
    pd_.ekf_not_empty = true;
  }
  //  inertia estimate
  b_M[0] = M[0];
  b_M[1] = M[1];
  b_M[2] = M[2];
  b_M[3] = sample_t;
  pd_.ekf.predict(b_M);
  //  预测下一时刻的状态
  if ((!pd_.ekf.pIsSetStateCovariance) ||
      (pd_.ekf.pSqrtStateCovarianceScalar != -1.0)) {
    for (b_i = 0; b_i < 36; b_i++) {
      pd_.ekf.pSqrtStateCovariance[b_i] =
          pd_.ekf.pSqrtStateCovarianceScalar * static_cast<double>(iv[b_i]);
    }
    pd_.ekf.pIsSetStateCovariance = true;
    pd_.ekf.pSqrtStateCovarianceScalar = -1.0;
  }
  if (pd_.ekf.pIsFirstCallCorrect) {
    if (!pd_.ekf.pIsValidMeasurementFcn) {
      pd_.ekf.pIsValidMeasurementFcn = true;
    }
    pd_.ekf.pIsFirstCallCorrect = false;
  }
  if ((!pd_.ekf.pIsSetMeasurementNoise) ||
      (pd_.ekf.pSqrtMeasurementNoiseScalar != -1.0)) {
    for (b_i = 0; b_i < 9; b_i++) {
      pd_.ekf.pSqrtMeasurementNoise[b_i] =
          pd_.ekf.pSqrtMeasurementNoiseScalar * static_cast<double>(b[b_i]);
    }
    pd_.ekf.pIsSetMeasurementNoise = true;
    pd_.ekf.pSqrtMeasurementNoiseScalar = -1.0;
  }
  std::copy(&pd_.ekf.pSqrtMeasurementNoise[0],
            &pd_.ekf.pSqrtMeasurementNoise[9], &Rsqrt[0]);
  for (int i{0}; i < 6; i++) {
    stateEstimate[i] = pd_.ekf.pState[i];
  }
  std::copy(&pd_.ekf.pSqrtStateCovariance[0], &pd_.ekf.pSqrtStateCovariance[36],
            &dv[0]);
  for (int j{0}; j < 3; j++) {
    for (int i{0}; i < 6; i++) {
      aoffset = i * 6;
      s = 0.0;
      for (k = 0; k < 6; k++) {
        s += dv[aoffset + k] * static_cast<double>(b_iv[k * 3 + j]);
      }
      c_M[i + 9 * j] = s;
    }
    c_M[9 * j + 6] = Rsqrt[j];
    c_M[9 * j + 7] = Rsqrt[j + 3];
    c_M[9 * j + 8] = Rsqrt[j + 6];
    tau[j] = 0.0;
    work[j] = 0.0;
  }
  for (int i{0}; i < 3; i++) {
    double atmp;
    int ii;
    int knt;
    ii = i * 9 + i;
    atmp = c_M[ii];
    aoffset = ii + 2;
    tau[i] = 0.0;
    s = coder::internal::blas::b_xnrm2(8 - i, c_M, ii + 2);
    if (s != 0.0) {
      double beta1;
      d = c_M[ii];
      beta1 = rt_hypotd_snf(d, s);
      if (d >= 0.0) {
        beta1 = -beta1;
      }
      if (std::abs(beta1) < 1.0020841800044864E-292) {
        knt = 0;
        b_i = (ii - i) + 9;
        do {
          knt++;
          for (k = aoffset; k <= b_i; k++) {
            c_M[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));
        beta1 = rt_hypotd_snf(
            atmp, coder::internal::blas::b_xnrm2(8 - i, c_M, ii + 2));
        if (atmp >= 0.0) {
          beta1 = -beta1;
        }
        tau[i] = (beta1 - atmp) / beta1;
        s = 1.0 / (atmp - beta1);
        for (k = aoffset; k <= b_i; k++) {
          c_M[k - 1] *= s;
        }
        for (k = 0; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        atmp = beta1;
      } else {
        tau[i] = (beta1 - d) / beta1;
        s = 1.0 / (d - beta1);
        b_i = (ii - i) + 9;
        for (k = aoffset; k <= b_i; k++) {
          c_M[k - 1] *= s;
        }
        atmp = beta1;
      }
    }
    c_M[ii] = atmp;
    if (i + 1 < 3) {
      int lastc;
      int lastv;
      c_M[ii] = 1.0;
      if (tau[i] != 0.0) {
        boolean_T exitg2;
        lastv = 9 - i;
        aoffset = (ii - i) + 8;
        while ((lastv > 0) && (c_M[aoffset] == 0.0)) {
          lastv--;
          aoffset--;
        }
        lastc = 1 - i;
        exitg2 = false;
        while ((!exitg2) && (lastc + 1 > 0)) {
          int exitg1;
          aoffset = (ii + lastc * 9) + 9;
          knt = aoffset;
          do {
            exitg1 = 0;
            if (knt + 1 <= aoffset + lastv) {
              if (c_M[knt] != 0.0) {
                exitg1 = 1;
              } else {
                knt++;
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
        k = ii + 10;
        if (lastc + 1 != 0) {
          std::memset(&work[0], 0,
                      static_cast<unsigned int>(lastc + 1) * sizeof(double));
          b_i = (ii + 9 * lastc) + 10;
          for (int j{k}; j <= b_i; j += 9) {
            s = 0.0;
            i1 = (j + lastv) - 1;
            for (knt = j; knt <= i1; knt++) {
              s += c_M[knt - 1] * c_M[(ii + knt) - j];
            }
            aoffset = div_nde_s32_floor((j - ii) - 10, 9);
            work[aoffset] += s;
          }
        }
        if (!(-tau[i] == 0.0)) {
          aoffset = ii;
          for (int j{0}; j <= lastc; j++) {
            d = work[j];
            if (d != 0.0) {
              s = d * -tau[i];
              b_i = aoffset + 10;
              i1 = lastv + aoffset;
              for (knt = b_i; knt <= i1 + 9; knt++) {
                c_M[knt - 1] += c_M[((ii + knt) - aoffset) - 10] * s;
              }
            }
            aoffset += 9;
          }
        }
      }
      c_M[ii] = atmp;
    }
  }
  for (int j{0}; j < 3; j++) {
    for (int i{0}; i <= j; i++) {
      R[i + 3 * j] = c_M[i + 9 * j];
    }
    b_i = j + 2;
    if (b_i <= 3) {
      std::memset(&R[(j * 3 + b_i) + -1], 0,
                  static_cast<unsigned int>(-b_i + 4) * sizeof(double));
    }
  }
  tau[0] = Omega[0] - stateEstimate[0];
  tau[1] = Omega[1] - stateEstimate[1];
  tau[2] = Omega[2] - stateEstimate[2];
  for (b_i = 0; b_i < 6; b_i++) {
    for (i1 = 0; i1 < 6; i1++) {
      d = 0.0;
      for (aoffset = 0; aoffset < 6; aoffset++) {
        d += dv[b_i + 6 * aoffset] * dv[i1 + 6 * aoffset];
      }
      dv1[b_i + 6 * i1] = d;
    }
    for (i1 = 0; i1 < 3; i1++) {
      d = 0.0;
      for (aoffset = 0; aoffset < 6; aoffset++) {
        d +=
            dv1[b_i + 6 * aoffset] * static_cast<double>(iv1[aoffset + 6 * i1]);
      }
      y[b_i + 6 * i1] = d;
    }
  }
  for (b_i = 0; b_i < 3; b_i++) {
    b_R[3 * b_i] = R[b_i];
    b_R[3 * b_i + 1] = R[b_i + 3];
    b_R[3 * b_i + 2] = R[b_i + 6];
  }
  coder::matlabshared::tracking::internal::EKFCorrectorAdditive::
      correctStateAndSqrtCovariance(stateEstimate, dv, tau, y, b_R, b_b, Rsqrt);
  for (int i{0}; i < 6; i++) {
    pd_.ekf.pState[i] = stateEstimate[i];
  }
  std::copy(&dv[0], &dv[36], &pd_.ekf.pSqrtStateCovariance[0]);
  pd_.ekf.pIsSetStateCovariance = true;
  pd_.ekf.pSqrtStateCovarianceScalar = -1.0;
  if (!pd_.ekf.pIsInitialized) {
    pd_.ekf.pIsDistributionsSetup = true;
  }
  //  根据实际测量值更新状态估计
  //  获取更新后的状态和协方差
  for (int i{0}; i < 6; i++) {
    stateEstimate[i] = pd_.ekf.pState[i];
  }
  if ((!pd_.ekf.pIsSetStateCovariance) ||
      (pd_.ekf.pSqrtStateCovarianceScalar != -1.0)) {
    for (b_i = 0; b_i < 36; b_i++) {
      pd_.ekf.pSqrtStateCovariance[b_i] =
          pd_.ekf.pSqrtStateCovarianceScalar * static_cast<double>(iv[b_i]);
    }
    pd_.ekf.pIsSetStateCovariance = true;
    pd_.ekf.pSqrtStateCovarianceScalar = -1.0;
  }
  std::copy(&pd_.ekf.pSqrtStateCovariance[0], &pd_.ekf.pSqrtStateCovariance[36],
            &dv[0]);
  for (b_i = 0; b_i < 6; b_i++) {
    for (i1 = 0; i1 < 6; i1++) {
      d = 0.0;
      for (aoffset = 0; aoffset < 6; aoffset++) {
        d += pd_.ekf.pSqrtStateCovariance[b_i + 6 * aoffset] *
             dv[i1 + 6 * aoffset];
      }
      dv1[b_i + 6 * i1] = d;
    }
    out[b_i] = stateEstimate[b_i];
  }
  std::copy(&dv1[0], &dv1[36], &out[6]);
}

//
// Arguments    : const double x[6]
//                const double p_[4]
//                double out[6]
// Return Type  : void
//
void stateTransitionFcn(const double x[6], const double p_[4], double out[6])
{
  double inertia[9];
  double dotOmega[3];
  double B_idx_0;
  double B_idx_1;
  double B_idx_2;
  std::memset(&inertia[0], 0, 9U * sizeof(double));
  inertia[0] = x[3];
  inertia[4] = x[4];
  inertia[8] = x[5];
  B_idx_0 = x[0];
  B_idx_1 = x[1];
  B_idx_2 = x[2];
  for (int i{0}; i < 3; i++) {
    dotOmega[i] = (inertia[i] * B_idx_0 + inertia[i + 3] * B_idx_1) +
                  inertia[i + 6] * B_idx_2;
  }
  B_idx_0 = p_[0] - (x[1] * dotOmega[2] - dotOmega[1] * x[2]);
  B_idx_1 = p_[1] - (dotOmega[0] * x[2] - x[0] * dotOmega[2]);
  B_idx_2 = p_[2] - (x[0] * dotOmega[1] - dotOmega[0] * x[1]);
  inertia[1] = 0.0 / x[3];
  inertia[2] = 0.0 / x[3];
  inertia[4] = x[4] - inertia[1] * 0.0;
  inertia[5] = 0.0 - inertia[2] * 0.0;
  inertia[7] = 0.0 - inertia[1] * 0.0;
  inertia[8] = x[5] - inertia[2] * 0.0;
  inertia[5] /= inertia[4];
  inertia[8] -= inertia[5] * inertia[7];
  dotOmega[1] = B_idx_1 - B_idx_0 * inertia[1];
  dotOmega[2] = (B_idx_2 - B_idx_0 * inertia[2]) - dotOmega[1] * inertia[5];
  dotOmega[2] /= inertia[8];
  dotOmega[0] = B_idx_0 - dotOmega[2] * 0.0;
  dotOmega[1] -= dotOmega[2] * inertia[7];
  dotOmega[1] /= inertia[4];
  dotOmega[0] -= dotOmega[1] * 0.0;
  dotOmega[0] /= x[3];
  out[0] = x[0] + dotOmega[0] * p_[3];
  out[3] = x[3];
  out[1] = x[1] + dotOmega[1] * p_[3];
  out[4] = x[4];
  out[2] = x[2] + dotOmega[2] * p_[3];
  out[5] = x[5];
}

//
// File trailer for InertiaEstimatorEKF.cpp
//
// [EOF]
//

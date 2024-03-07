//
// File: trackingEKF.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 01-Mar-2024 18:43:15
//

#ifndef TRACKINGEKF_H
#define TRACKINGEKF_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
class trackingEKF {
public:
  void predict(const double varargin_1[4]);
  trackingEKF();
  ~trackingEKF();
  double pState[6];
  double pSqrtStateCovariance[36];
  double pSqrtStateCovarianceScalar;
  boolean_T pIsSetStateCovariance;
  double pSqrtProcessNoise[36];
  double pSqrtProcessNoiseScalar;
  boolean_T pIsSetProcessNoise;
  double pSqrtMeasurementNoise[9];
  double pSqrtMeasurementNoiseScalar;
  boolean_T pIsSetMeasurementNoise;
  boolean_T pIsValidStateTransitionFcn;
  boolean_T pIsValidMeasurementFcn;
  boolean_T pIsFirstCallPredict;
  boolean_T pIsFirstCallCorrect;
  boolean_T pIsDistributionsSetup;
  boolean_T pIsInitialized;
  boolean_T IsLastJacobianInitialized;

protected:
  double pJacobian[36];
};

} // namespace coder

#endif
//
// File trailer for trackingEKF.h
//
// [EOF]
//

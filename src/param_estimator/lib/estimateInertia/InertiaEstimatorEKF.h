//
// File: InertiaEstimatorEKF.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 01-Mar-2024 18:43:15
//

#ifndef INERTIAESTIMATOREKF_H
#define INERTIAESTIMATOREKF_H

// Include Files
#include "estimateInertia_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class InertiaEstimatorEKF {
public:
  InertiaEstimatorEKF();
  ~InertiaEstimatorEKF();
  void estimateInertia(const double M[3], double sample_t,
                       const double Omega[3], double out[42]);
  estimateInertiaStackData *getStackData();

private:
  estimateInertiaPersistentData pd_;
  estimateInertiaStackData SD_;
};

// Function Declarations
void stateTransitionFcn(const double x[6], const double p_[4], double out[6]);

#endif
//
// File trailer for InertiaEstimatorEKF.h
//
// [EOF]
//

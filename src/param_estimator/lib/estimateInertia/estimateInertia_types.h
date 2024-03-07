//
// File: estimateInertia_types.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 01-Mar-2024 18:43:15
//

#ifndef ESTIMATEINERTIA_TYPES_H
#define ESTIMATEINERTIA_TYPES_H

// Include Files
#include "rtwtypes.h"
#include "trackingEKF.h"

// Type Definitions
struct estimateInertiaPersistentData {
  coder::trackingEKF ekf;
  boolean_T ekf_not_empty;
};

struct estimateInertiaStackData {
  estimateInertiaPersistentData *pd;
};

#endif
//
// File trailer for estimateInertia_types.h
//
// [EOF]
//

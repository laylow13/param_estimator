//
// File: _coder_estimateInertia_api.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 01-Mar-2024 18:43:15
//

#ifndef _CODER_ESTIMATEINERTIA_API_H
#define _CODER_ESTIMATEINERTIA_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void estimateInertia(real_T M[3], real_T sample_t, real_T Omega[3],
                     real_T out[42]);

void estimateInertia_api(const mxArray *const prhs[3], const mxArray **plhs);

void estimateInertia_atexit();

void estimateInertia_initialize();

void estimateInertia_terminate();

void estimateInertia_xil_shutdown();

void estimateInertia_xil_terminate();

#endif
//
// File trailer for _coder_estimateInertia_api.h
//
// [EOF]
//

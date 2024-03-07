//
// File: _coder_estimateInertia_mex.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 01-Mar-2024 18:43:15
//

#ifndef _CODER_ESTIMATEINERTIA_MEX_H
#define _CODER_ESTIMATEINERTIA_MEX_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_estimateInertia_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                        int32_T nrhs, const mxArray *prhs[3]);

#endif
//
// File trailer for _coder_estimateInertia_mex.h
//
// [EOF]
//

//
// File: _coder_estimateInertia_mex.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 01-Mar-2024 18:43:15
//

// Include Files
#include "_coder_estimateInertia_mex.h"
#include "_coder_estimateInertia_api.h"

// Function Definitions
//
// Arguments    : int32_T nlhs
//                mxArray *plhs[]
//                int32_T nrhs
//                const mxArray *prhs[]
// Return Type  : void
//
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&estimateInertia_atexit);
  // Module initialization.
  estimateInertia_initialize();
  // Dispatch the entry-point.
  unsafe_estimateInertia_mexFunction(nlhs, plhs, nrhs, prhs);
  // Module termination.
  estimateInertia_terminate();
}

//
// Arguments    : void
// Return Type  : emlrtCTX
//
emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr, "GBK", true);
  return emlrtRootTLSGlobal;
}

//
// Arguments    : int32_T nlhs
//                mxArray *plhs[1]
//                int32_T nrhs
//                const mxArray *prhs[3]
// Return Type  : void
//
void unsafe_estimateInertia_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                        int32_T nrhs, const mxArray *prhs[3])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[3];
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        15, "estimateInertia");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 15,
                        "estimateInertia");
  }
  // Call the function.
  b_prhs[0] = prhs[0];
  b_prhs[1] = prhs[1];
  b_prhs[2] = prhs[2];
  estimateInertia_api(b_prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

//
// File trailer for _coder_estimateInertia_mex.cpp
//
// [EOF]
//

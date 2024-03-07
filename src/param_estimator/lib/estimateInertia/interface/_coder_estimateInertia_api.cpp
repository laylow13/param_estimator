//
// File: _coder_estimateInertia_api.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 01-Mar-2024 18:43:15
//

// Include Files
#include "_coder_estimateInertia_api.h"
#include "_coder_estimateInertia_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131642U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "estimateInertia",                                    // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *sample_t,
                                 const char_T *identifier);

static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T (*c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3];

static real_T d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *M,
                                 const char_T *identifier))[3];

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[3];

static const mxArray *emlrt_marshallOut(const real_T u[42]);

// Function Definitions
//
// Arguments    : const emlrtStack &sp
//                const mxArray *sample_t
//                const char_T *identifier
// Return Type  : real_T
//
static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *sample_t,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(sample_t), &thisId);
  emlrtDestroyArray(&sample_t);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T
//
static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = d_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[3]
//
static real_T (*c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims{3};
  real_T(*ret)[3];
  int32_T i;
  boolean_T b{false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T
//
static real_T d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *M
//                const char_T *identifier
// Return Type  : real_T (*)[3]
//
static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *M,
                                 const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[3];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(M), &thisId);
  emlrtDestroyArray(&M);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[3]
//
static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[3]
{
  real_T(*y)[3];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u[42]
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u[42])
{
  static const int32_T i{0};
  static const int32_T i1{42};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const mxArray * const prhs[3]
//                const mxArray **plhs
// Return Type  : void
//
void estimateInertia_api(const mxArray *const prhs[3], const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*out)[42];
  real_T(*M)[3];
  real_T(*Omega)[3];
  real_T sample_t;
  st.tls = emlrtRootTLSGlobal;
  out = (real_T(*)[42])mxMalloc(sizeof(real_T[42]));
  // Marshall function inputs
  M = emlrt_marshallIn(st, emlrtAlias(prhs[0]), "M");
  sample_t = b_emlrt_marshallIn(st, emlrtAliasP(prhs[1]), "sample_t");
  Omega = emlrt_marshallIn(st, emlrtAlias(prhs[2]), "Omega");
  // Invoke the target function
  estimateInertia(*M, sample_t, *Omega, *out);
  // Marshall function outputs
  *plhs = emlrt_marshallOut(*out);
}

//
// Arguments    : void
// Return Type  : void
//
void estimateInertia_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  estimateInertia_xil_terminate();
  estimateInertia_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void estimateInertia_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void estimateInertia_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// File trailer for _coder_estimateInertia_api.cpp
//
// [EOF]
//

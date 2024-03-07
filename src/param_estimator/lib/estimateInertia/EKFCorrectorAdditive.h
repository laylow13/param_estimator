//
// File: EKFCorrectorAdditive.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 01-Mar-2024 18:43:15
//

#ifndef EKFCORRECTORADDITIVE_H
#define EKFCORRECTORADDITIVE_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace matlabshared {
namespace tracking {
namespace internal {
class EKFCorrectorAdditive {
public:
  static void correctStateAndSqrtCovariance(
      double x[6], double S[36], const double residue[3], const double Pxy[18],
      const double Sy[9], const double H[18], const double Rsqrt[9]);
};

} // namespace internal
} // namespace tracking
} // namespace matlabshared
} // namespace coder

#endif
//
// File trailer for EKFCorrectorAdditive.h
//
// [EOF]
//

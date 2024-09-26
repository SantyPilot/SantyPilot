//
// File: ut_mweights.h
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 16:03:50
//
#ifndef UT_MWEIGHTS_H
#define UT_MWEIGHTS_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "ukf_update2_types.h"

namespace ukf_update2_ns {
// Function Declarations
extern void ut_mweights(double alpha, double beta, double kappa, double WM[27],
  double W[729], double *c);
}

#endif

//
// File trailer for ut_mweights.h
//
// [EOF]
//

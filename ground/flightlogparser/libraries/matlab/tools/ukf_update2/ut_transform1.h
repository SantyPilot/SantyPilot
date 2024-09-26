//
// File: ut_transform1.h
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 16:03:50
//
#ifndef UT_TRANSFORM1_H
#define UT_TRANSFORM1_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "ukf_update2_types.h"

// Function Declarations
extern void ut_transform1(const double M[13], const double P[169], const double
  g_param[3], const double tr_param[4], double mu[10], double S[100], double
  C_data[], int C_size[2]);

#endif

//
// File trailer for ut_transform1.h
//
// [EOF]
//

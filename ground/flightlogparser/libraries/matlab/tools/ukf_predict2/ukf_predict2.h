//
// File: ukf_predict2.h
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 12:20:49
//
#ifndef UKF_PREDICT2_H
#define UKF_PREDICT2_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "ukf_predict2_types.h"

// Function Declarations
extern void ukf_predict2(double M[13], double P[169], const double Q[169], const
  double f_param[7], double alpha, double beta, double kappa, double mat, double
  D_data[], int D_size[2]);

#endif

//
// File trailer for ukf_predict2.h
//
// [EOF]
//

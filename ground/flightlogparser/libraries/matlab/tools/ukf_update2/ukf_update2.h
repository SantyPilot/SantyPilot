//
// File: ukf_update2.h
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 16:03:50
//
#ifndef UKF_UPDATE2_H
#define UKF_UPDATE2_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "ukf_update2_types.h"

// Function Declarations
extern void ukf_update2(double M[13], double P[169], const double Y[10], const
  double R[100], const double h_param[3], double alpha, double beta, double
  kappa, double mat, double K_data[], int K_size[2]);

#endif

//
// File trailer for ukf_update2.h
//
// [EOF]
//

//
// File: ukf_track_h_ins.h
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 16:03:50
//
#ifndef UKF_TRACK_H_INS_H
#define UKF_TRACK_H_INS_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "ukf_update2_types.h"

// Function Declarations
extern void ukf_track_h_ins(const double x_data[], const double hparam[3],
  double y[10]);

#endif

//
// File trailer for ukf_track_h_ins.h
//
// [EOF]
//

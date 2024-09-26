//
// File: ukf_track_h_ins.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 14:58:44
//

// Include Files
#include "ukf_track_h_ins.h"

// Function Definitions

//
// Arguments    : const double x[13]
//                const double hparam[3]
//                double y[10]
// Return Type  : void
//
void ukf_track_h_ins(const double x[13], const double hparam[3], double y[10])
{
  //  first six outputs are P and V
  y[0] = x[0];
  y[1] = x[1];
  y[2] = x[2];
  y[3] = x[3];
  y[4] = x[4];
  y[5] = x[5];

  //  Bb= Rbe * Be
  y[6] = ((((x[6] * x[6] + x[7] * x[7]) - x[8] * x[8]) - x[9] * x[9]) * hparam[0]
          + 2.0 * (x[7] * x[8] + x[6] * x[9]) * hparam[1]) + 2.0 * (x[7] * x[9]
    - x[6] * x[8]) * hparam[2];
  y[7] = (2.0 * (x[7] * x[8] - x[6] * x[9]) * hparam[0] + (((x[6] * x[6] - x[7] *
             x[7]) + x[8] * x[8]) - x[9] * x[9]) * hparam[1]) + 2.0 * (x[8] * x
    [9] + x[6] * x[7]) * hparam[2];
  y[8] = (2.0 * (x[7] * x[9] + x[6] * x[8]) * hparam[0] + 2.0 * (x[8] * x[9] -
           x[6] * x[7]) * hparam[1]) + (((x[6] * x[6] - x[7] * x[7]) - x[8] * x
    [8]) + x[9] * x[9]) * hparam[2];

  //  Alt = -Pz
  y[9] = -x[2];
}

//
// File trailer for ukf_track_h_ins.cpp
//
// [EOF]
//

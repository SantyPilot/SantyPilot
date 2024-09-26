//
// File: ukf_track_h_ins.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 16:03:50
//

// Include Files
#include "ukf_update2.h"
#include "ukf_track_h_ins.h"

namespace ukf_update2_ns {
// Function Definitions

//
// Arguments    : const double x_data[]
//                const double hparam[3]
//                double y[10]
// Return Type  : void
//
void ukf_track_h_ins(const double x_data[], const double hparam[3], double y[10])
{
  //  first six outputs are P and V
  y[0] = x_data[0];
  y[1] = x_data[1];
  y[2] = x_data[2];
  y[3] = x_data[3];
  y[4] = x_data[4];
  y[5] = x_data[5];

  //  Bb= Rbe * Be
  y[6] = ((((x_data[6] * x_data[6] + x_data[7] * x_data[7]) - x_data[8] *
            x_data[8]) - x_data[9] * x_data[9]) * hparam[0] + 2.0 * (x_data[7] *
           x_data[8] + x_data[6] * x_data[9]) * hparam[1]) + 2.0 * (x_data[7] *
    x_data[9] - x_data[6] * x_data[8]) * hparam[2];
  y[7] = (2.0 * (x_data[7] * x_data[8] - x_data[6] * x_data[9]) * hparam[0] +
          (((x_data[6] * x_data[6] - x_data[7] * x_data[7]) + x_data[8] *
            x_data[8]) - x_data[9] * x_data[9]) * hparam[1]) + 2.0 * (x_data[8] *
    x_data[9] + x_data[6] * x_data[7]) * hparam[2];
  y[8] = (2.0 * (x_data[7] * x_data[9] + x_data[6] * x_data[8]) * hparam[0] +
          2.0 * (x_data[8] * x_data[9] - x_data[6] * x_data[7]) * hparam[1]) +
    (((x_data[6] * x_data[6] - x_data[7] * x_data[7]) - x_data[8] * x_data[8]) +
     x_data[9] * x_data[9]) * hparam[2];

  //  Alt = -Pz
  y[9] = -x_data[2];
}

} // ukf_update2
//
// File trailer for ukf_track_h_ins.cpp
//
// [EOF]
//

//
// File: ukf_track_f_ins.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 12:20:49
//

// Include Files
#include "ukf_predict2.h"
#include "ukf_track_f_ins.h"
#include "ins_state_eq.h"

// Function Definitions

//
// Arguments    : const double x_data[]
//                const double param[7]
//                double x_n[13]
// Return Type  : void
//
void ukf_track_f_ins(const double x_data[], const double param[7], double x_n[13])
{
  double dT2;
  double wx;
  double wy;
  double wz;
  double K1[13];
  int i2;
  double x[13];
  double K2[13];
  double K3[13];
  double dv0[13];
  dT2 = param[0] / 2.0;
  wx = param[1] - x_data[10];
  wy = param[2] - x_data[11];
  wz = param[3] - x_data[12];

  //  pdot = v
  K1[0] = x_data[3];
  K1[1] = x_data[4];
  K1[2] = x_data[5];

  //  vdot = reb * a
  K1[3] = ((((x_data[6] * x_data[6] + x_data[7] * x_data[7]) - x_data[8] *
             x_data[8]) - x_data[9] * x_data[9]) * param[4] + 2.0 * (x_data[7] *
            x_data[8] - x_data[6] * x_data[9]) * param[5]) + 2.0 * (x_data[7] *
    x_data[9] + x_data[6] * x_data[8]) * param[6];
  K1[4] = ((((x_data[6] * x_data[6] - x_data[7] * x_data[7]) + x_data[8] *
             x_data[8]) - x_data[9] * x_data[9]) * param[5] + 2.0 * (x_data[7] *
            x_data[8] + x_data[6] * x_data[9]) * param[4]) + 2.0 * (x_data[8] *
    x_data[9] - x_data[6] * x_data[7]) * param[6];
  K1[5] = (((((x_data[6] * x_data[6] - x_data[7] * x_data[7]) - x_data[8] *
              x_data[8]) + x_data[9] * x_data[9]) * param[6] + 2.0 * (x_data[7] *
             x_data[9] - x_data[6] * x_data[8]) * param[4]) + 2.0 * (x_data[8] *
            x_data[9] + x_data[6] * x_data[7]) * param[5]) + 9.81;

  //  qdot = q * w
  K1[6] = ((-x_data[7] * wx - x_data[8] * wy) - x_data[9] * wz) / 2.0;
  K1[7] = ((x_data[6] * wx - x_data[9] * wy) + x_data[8] * wz) / 2.0;
  K1[8] = ((x_data[9] * wx + x_data[6] * wy) - x_data[7] * wz) / 2.0;
  K1[9] = ((-x_data[8] * wx + x_data[7] * wy) + x_data[6] * wz) / 2.0;

  //  gyro bias guess 0
  K1[10] = 0.0;
  K1[11] = 0.0;
  K1[12] = 0.0;
  for (i2 = 0; i2 < 13; i2++) {
    x[i2] = x_data[i2] + dT2 * K1[i2];
  }

  ins_state_eq(x, *(double (*)[6])&param[1], K2);
  for (i2 = 0; i2 < 13; i2++) {
    x[i2] = x_data[i2] + dT2 * K2[i2];
  }

  ins_state_eq(x, *(double (*)[6])&param[1], K3);

  //  X = Xlast + dT2 * K4;
  for (i2 = 0; i2 < 13; i2++) {
    x[i2] = x_data[i2] + param[0] * K3[i2];
  }

  ins_state_eq(x, *(double (*)[6])&param[1], dv0);
  for (i2 = 0; i2 < 13; i2++) {
    x_n[i2] = x_data[i2] + param[0] * (((K1[i2] + 2.0 * K2[i2]) + 2.0 * K3[i2])
      + dv0[i2]) * 0.16666666666666666;
  }

  //  normalize quaternion
  dT2 = ((x_n[6] * x_n[6] + x_n[7] * x_n[7]) + x_n[8] * x_n[8]) + x_n[9] * x_n[9];
  dT2 = 1.0 / (dT2 * dT2);
  x_n[6] *= dT2;
  x_n[7] *= dT2;
  x_n[8] *= dT2;
  x_n[9] *= dT2;
}

//
// File trailer for ukf_track_f_ins.cpp
//
// [EOF]
//

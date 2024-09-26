//
// File: ins_state_eq.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 12:20:49
//

// Include Files
#include "ukf_predict2.h"
#include "ins_state_eq.h"

// Function Definitions

//
// Arguments    : const double X[13]
//                const double U[6]
//                double Xdot[13]
// Return Type  : void
//
void ins_state_eq(const double X[13], const double U[6], double Xdot[13])
{
  double wx;
  double wy;
  double wz;
  wx = U[0] - X[10];
  wy = U[1] - X[11];
  wz = U[2] - X[12];

  //  pdot = v
  Xdot[0] = X[3];
  Xdot[1] = X[4];
  Xdot[2] = X[5];

  //  vdot = reb * a
  Xdot[3] = ((((X[6] * X[6] + X[7] * X[7]) - X[8] * X[8]) - X[9] * X[9]) * U[3]
             + 2.0 * (X[7] * X[8] - X[6] * X[9]) * U[4]) + 2.0 * (X[7] * X[9] +
    X[6] * X[8]) * U[5];
  Xdot[4] = ((((X[6] * X[6] - X[7] * X[7]) + X[8] * X[8]) - X[9] * X[9]) * U[4]
             + 2.0 * (X[7] * X[8] + X[6] * X[9]) * U[3]) + 2.0 * (X[8] * X[9] -
    X[6] * X[7]) * U[5];
  Xdot[5] = (((((X[6] * X[6] - X[7] * X[7]) - X[8] * X[8]) + X[9] * X[9]) * U[5]
              + 2.0 * (X[7] * X[9] - X[6] * X[8]) * U[3]) + 2.0 * (X[8] * X[9] +
              X[6] * X[7]) * U[4]) + 9.81;

  //  qdot = q * w
  Xdot[6] = ((-X[7] * wx - X[8] * wy) - X[9] * wz) / 2.0;
  Xdot[7] = ((X[6] * wx - X[9] * wy) + X[8] * wz) / 2.0;
  Xdot[8] = ((X[9] * wx + X[6] * wy) - X[7] * wz) / 2.0;
  Xdot[9] = ((-X[8] * wx + X[7] * wy) + X[6] * wz) / 2.0;

  //  gyro bias guess 0
  Xdot[10] = 0.0;
  Xdot[11] = 0.0;
  Xdot[12] = 0.0;
}

//
// File trailer for ins_state_eq.cpp
//
// [EOF]
//

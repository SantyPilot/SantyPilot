//
// File: ukf_update2.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 16:03:50
//

// Include Files
#include "ukf_update2.h"
#include "mrdivide.h"
#include "ut_transform1.h"

// Function Definitions

//
// Do transform and make the update
// Arguments    : double M[13]
//                double P[169]
//                const double Y[10]
//                const double R[100]
//                const double h_param[3]
//                double alpha
//                double beta
//                double kappa
//                double mat
//                double K_data[]
//                int K_size[2]
// Return Type  : void
//
void ukf_update2(double M[13], double P[169], const double Y[10], const double
                 R[100], const double h_param[3], double alpha, double beta,
                 double kappa, double mat, double K_data[], int K_size[2])
{
  double b_alpha[4];
  double MU[10];
  double S[100];
  int ic;
  double C_data[13];
  int ar;
  int ib;
  int ia;
  double y_data[130];
  int cr;
  int br;
  double b_C_data[169];
  double b_data[130];
  b_alpha[0] = alpha;
  b_alpha[1] = beta;
  b_alpha[2] = kappa;
  b_alpha[3] = mat;
  ut_transform1(M, P, h_param, b_alpha, MU, S, K_data, K_size);
  for (ic = 0; ic < 100; ic++) {
    S[ic] += R[ic];
  }

  mrdivide(K_data, K_size, S);
  for (ic = 0; ic < 10; ic++) {
    MU[ic] = Y[ic] - MU[ic];
  }

  memset(&C_data[0], 0, 13U * sizeof(double));
  for (ic = 1; ic < 14; ic++) {
    C_data[ic - 1] = 0.0;
  }

  ar = -1;
  for (ib = 0; ib + 1 < 11; ib++) {
    if (MU[ib] != 0.0) {
      ia = ar;
      for (ic = 0; ic + 1 < 14; ic++) {
        ia++;
        C_data[ic] += MU[ib] * K_data[ia];
      }
    }

    ar += 13;
  }

  for (ic = 0; ic < 13; ic++) {
    M[ic] += C_data[ic];
  }

  memset(&y_data[0], 0, 130U * sizeof(double));
  for (cr = 0; cr <= 117; cr += 13) {
    for (ic = cr; ic + 1 <= cr + 13; ic++) {
      y_data[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= 117; cr += 13) {
    ar = -1;
    for (ib = br; ib + 1 <= br + 10; ib++) {
      if (S[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 13; ic++) {
          ia++;
          y_data[ic] += S[ib] * K_data[ia];
        }
      }

      ar += 13;
    }

    br += 10;
  }

  for (ic = 0; ic < 13; ic++) {
    for (ar = 0; ar < 10; ar++) {
      b_data[ar + 10 * ic] = K_data[ic + K_size[0] * ar];
    }

    memset(&b_C_data[ic * 13], 0, 13U * sizeof(double));
  }

  for (cr = 0; cr <= 156; cr += 13) {
    for (ic = cr; ic + 1 <= cr + 13; ic++) {
      b_C_data[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= 156; cr += 13) {
    ar = -1;
    for (ib = br; ib + 1 <= br + 10; ib++) {
      if (b_data[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 13; ic++) {
          ia++;
          b_C_data[ic] += b_data[ib] * y_data[ia];
        }
      }

      ar += 13;
    }

    br += 10;
  }

  for (ic = 0; ic < 169; ic++) {
    P[ic] -= b_C_data[ic];
  }
}

//
// File trailer for ukf_update2.cpp
//
// [EOF]
//

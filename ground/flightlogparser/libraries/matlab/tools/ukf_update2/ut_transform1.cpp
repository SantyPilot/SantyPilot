//
// File: ut_transform1.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 16:03:50
//

// Include Files
#include "ukf_update2.h"
#include "ut_transform1.h"
#include "ukf_update2_emxutil.h"
#include "ukf_track_h_ins.h"
#include "ut_sigmas.h"
#include "ut_mweights.h"

// Function Definitions

//
// Arguments    : const double M[13]
//                const double P[169]
//                const double g_param[3]
//                const double tr_param[4]
//                double mu[10]
//                double S[100]
//                double C_data[]
//                int C_size[2]
// Return Type  : void
//
void ut_transform1(const double M[13], const double P[169], const double
                   g_param[3], const double tr_param[4], double mu[10], double
                   S[100], double C_data[], int C_size[2])
{
  double WM[27];
  double W[729];
  double c;
  double X_data[351];
  emxArray_real_T *Y;
  int i0;
  emxArray_real_T *b_Y;
  int ar;
  double b_X_data[13];
  double dv0[10];
  int ib;
  int ia;
  int k;
  double y[270];
  int cr;
  int ic;
  emxArray_real_T *b;
  int br;
  double y_data[351];

  // UT_TRANSFORM  Perform unscented transform
  //
  //  Syntax:
  //    [mu,S,C,X,Y,w] = UT_TRANSFORM(M,P,g,g_param,tr_param)
  //
  //  In:
  //    M - Random variable mean (Nx1 column vector)
  //    P - Random variable covariance (NxN pos.def. matrix)
  //    g - Transformation function of the form g(x,param) as
  //        matrix, inline function, function name or function reference
  //    g_param - Parameters of g               (optional, default empty)
  //    tr_param - Parameters of the transformation as:
  //        alpha = tr_param{1} - Transformation parameter      (optional)
  //        beta  = tr_param{2} - Transformation parameter      (optional)
  //        kappa = tr_param{3} - Transformation parameter      (optional)
  //        mat   = tr_param{4} - If 1 uses matrix form         (optional, default 0) 
  //        X     = tr_param{5} - Sigma points of x
  //        w     = tr_param{6} - Weights as cell array {mean-weights,cov-weights,c} 
  //
  //  Out:
  //    mu - Estimated mean of y
  //     S - Estimated covariance of y
  //     C - Estimated cross-covariance of x and y
  //     X - Sigma points of x
  //     Y - Sigma points of y
  //     w - Weights as cell array {mean-weights,cov-weights,c}
  //
  //  Description:
  //    ...
  //    For default values of parameters, see UT_WEIGHTS.
  //
  //  See also
  //    UT_WEIGHTS UT_MWEIGHTS UT_SIGMAS
  //  Copyright (C) 2006 Simo Sï¿½rkkï¿?%               2010 Jouni Hartikainen
  //
  //  $Id: ut_transform.m 482 2010-10-18 07:53:23Z jmjharti $
  //
  //  This software is distributed under the GNU General Public
  //  Licence (version 2 or later); please refer to the file
  //  Licence.txt, included with the software, for details.
  ukf_update2_ns::ut_mweights(tr_param[0], tr_param[1], tr_param[2], WM, W, &c);
  ukf_update2_ns::ut_sigmas(M, P, c, X_data);
  ukf_update2_ns::emxInit_real_T(&Y, 2);

  //
  //  Propagate through the function
  //
  i0 = Y->size[0] * Y->size[1];
  Y->size[0] = 10;
  Y->size[1] = 0;
  ukf_update2_ns::emxEnsureCapacity((emxArray__common *)Y, i0, sizeof(double));
  ukf_update2_ns::emxInit_real_T(&b_Y, 2);
  for (ar = 0; ar < 27; ar++) {
    memcpy(&b_X_data[0], &X_data[ar * 13], 13U * sizeof(double));
    ukf_track_h_ins(b_X_data, g_param, dv0);
    i0 = b_Y->size[0] * b_Y->size[1];
    b_Y->size[0] = 10;
    b_Y->size[1] = Y->size[1] + 1;
    ukf_update2_ns::emxEnsureCapacity((emxArray__common *)b_Y, i0, sizeof(double));
    ib = Y->size[1];
    for (i0 = 0; i0 < ib; i0++) {
      for (ia = 0; ia < 10; ia++) {
        b_Y->data[ia + b_Y->size[0] * i0] = Y->data[ia + Y->size[0] * i0];
      }
    }

    for (i0 = 0; i0 < 10; i0++) {
      b_Y->data[i0 + b_Y->size[0] * Y->size[1]] = dv0[i0];
    }

    i0 = Y->size[0] * Y->size[1];
    Y->size[0] = 10;
    Y->size[1] = b_Y->size[1];
    ukf_update2_ns::emxEnsureCapacity((emxArray__common *)Y, i0, sizeof(double));
    ib = b_Y->size[1];
    for (i0 = 0; i0 < ib; i0++) {
      for (ia = 0; ia < 10; ia++) {
        Y->data[ia + Y->size[0] * i0] = b_Y->data[ia + b_Y->size[0] * i0];
      }
    }
  }

  ukf_update2_ns::emxFree_real_T(&b_Y);
  if (Y->size[1] == 1) {
    for (i0 = 0; i0 < 10; i0++) {
      mu[i0] = 0.0;
      for (ia = 0; ia < 27; ia++) {
        c = mu[i0] + Y->data[i0 + 10 * ia] * WM[ia];
        mu[i0] = c;
      }
    }
  } else {
    memset(&mu[0], 0, 10U * sizeof(double));
    ar = -1;
    for (ib = 0; ib + 1 <= Y->size[1]; ib++) {
      if (WM[ib] != 0.0) {
        ia = ar;
        for (ic = 0; ic < 10; ic++) {
          ia++;
          c = mu[ic] + WM[ib] * Y->data[ia];
          mu[ic] = c;
        }
      }

      ar += 10;
    }
  }

  if (Y->size[1] == 1) {
    for (i0 = 0; i0 < 10; i0++) {
      for (ia = 0; ia < 27; ia++) {
        y[i0 + 10 * ia] = 0.0;
        for (ar = 0; ar < 27; ar++) {
          y[i0 + 10 * ia] += Y->data[i0 + 10 * ar] * W[ar + 27 * ia];
        }
      }
    }
  } else {
    k = Y->size[1];
    memset(&y[0], 0, 270U * sizeof(double));
    for (cr = 0; cr <= 261; cr += 10) {
      for (ic = cr; ic + 1 <= cr + 10; ic++) {
        y[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= 261; cr += 10) {
      ar = -1;
      i0 = br + k;
      for (ib = br; ib + 1 <= i0; ib++) {
        if (W[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 10; ic++) {
            ia++;
            y[ic] += W[ib] * Y->data[ia];
          }
        }

        ar += 10;
      }

      br += k;
    }
  }

  ukf_update2_ns::emxInit_real_T(&b, 2);
  i0 = b->size[0] * b->size[1];
  b->size[0] = Y->size[1];
  b->size[1] = 10;
  ukf_update2_ns::emxEnsureCapacity((emxArray__common *)b, i0, sizeof(double));
  for (i0 = 0; i0 < 10; i0++) {
    ib = Y->size[1];
    for (ia = 0; ia < ib; ia++) {
      b->data[ia + b->size[0] * i0] = Y->data[i0 + Y->size[0] * ia];
    }
  }

  if (b->size[0] == 1) {
    for (i0 = 0; i0 < 10; i0++) {
      for (ia = 0; ia < 10; ia++) {
        S[i0 + 10 * ia] = 0.0;
        for (ar = 0; ar < 27; ar++) {
          S[i0 + 10 * ia] += y[i0 + 10 * ar] * b->data[ar + 27 * ia];
        }
      }
    }
  } else {
    memset(&S[0], 0, 100U * sizeof(double));
    if (27 == b->size[0]) {
      for (i0 = 0; i0 < 10; i0++) {
        for (ia = 0; ia < 10; ia++) {
          S[i0 + 10 * ia] = 0.0;
          for (ar = 0; ar < 27; ar++) {
            S[i0 + 10 * ia] += y[i0 + 10 * ar] * b->data[ar + 27 * ia];
          }
        }
      }
    } else {
      for (cr = 0; cr <= 91; cr += 10) {
        for (ic = cr; ic + 1 <= cr + 10; ic++) {
          S[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= 91; cr += 10) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 27; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 10; ic++) {
              ia++;
              S[ic] += b->data[ib] * y[ia];
            }
          }

          ar += 10;
        }

        br += 27;
      }
    }
  }

  memset(&y_data[0], 0, 351U * sizeof(double));
  for (cr = 0; cr <= 338; cr += 13) {
    for (ic = cr; ic + 1 <= cr + 13; ic++) {
      y_data[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= 338; cr += 13) {
    ar = -1;
    for (ib = br; ib + 1 <= br + 27; ib++) {
      if (W[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 13; ic++) {
          ia++;
          y_data[ic] += W[ib] * X_data[ia];
        }
      }

      ar += 13;
    }

    br += 27;
  }

  i0 = b->size[0] * b->size[1];
  b->size[0] = Y->size[1];
  b->size[1] = 10;
  ukf_update2_ns::emxEnsureCapacity((emxArray__common *)b, i0, sizeof(double));
  for (i0 = 0; i0 < 10; i0++) {
    ib = Y->size[1];
    for (ia = 0; ia < ib; ia++) {
      b->data[ia + b->size[0] * i0] = Y->data[i0 + Y->size[0] * ia];
    }
  }

  ukf_update2_ns::emxFree_real_T(&Y);
  if (b->size[0] == 1) {
    C_size[0] = 13;
    C_size[1] = 10;
    for (i0 = 0; i0 < 13; i0++) {
      for (ia = 0; ia < 10; ia++) {
        C_data[i0 + 13 * ia] = 0.0;
        for (ar = 0; ar < 27; ar++) {
          C_data[i0 + 13 * ia] += y_data[i0 + 13 * ar] * b->data[ar + 27 * ia];
        }
      }
    }
  } else {
    C_size[0] = 13;
    C_size[1] = 10;
    memset(&C_data[0], 0, 130U * sizeof(double));
    for (cr = 0; cr <= 117; cr += 13) {
      for (ic = cr; ic + 1 <= cr + 13; ic++) {
        C_data[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= 117; cr += 13) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 27; ib++) {
        if (b->data[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 13; ic++) {
            ia++;
            C_data[ic] += b->data[ib] * y_data[ia];
          }
        }

        ar += 13;
      }

      br += 27;
    }
  }

  ukf_update2_ns::emxFree_real_T(&b);
}

//
// File trailer for ut_transform1.cpp
//
// [EOF]
//

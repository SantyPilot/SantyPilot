//
// File: ukf_predict2.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 12:20:49
//

// Include Files
#include "ukf_predict2.h"
#include "ukf_predict2_emxutil.h"
#include "ukf_track_f_ins.h"
#include "ut_sigmas.h"
#include "ut_mweights.h"

// Function Definitions

//
// In:
//    M - Nx1 mean state estimate of previous step
//    P - NxN state covariance of previous step
//    f - Dynamic model function as a matrix A defining
//        linear function a(x) = A*x, inline function,
//        function handle or name of function in
//        form a(x,param)                   (optional, default eye())
//    Q - Process noise of discrete model   (optional, default zero)
//    f_param - Parameters of f               (optional, default empty)
//    alpha - Transformation parameter      (optional)
//    beta  - Transformation parameter      (optional)
//    kappa - Transformation parameter      (optional)
//    mat   - If 1 uses matrix form         (optional, default 0)
//
//  Out:
//    M - Updated state mean
//    P - Updated state covariance
//
//  Description:
//    Perform additive form Unscented Kalman Filter prediction step.
// Arguments    : double M[13]
//                double P[169]
//                const double Q[169]
//                const double f_param[7]
//                double alpha
//                double beta
//                double kappa
//                double mat
//                double D_data[]
//                int D_size[2]
// Return Type  : void
//
void ukf_predict2(double M[13], double P[169], const double Q[169], const double
                  f_param[7], double alpha, double beta, double kappa, double,
                  double D_data[], int D_size[2])
{
  double WM[27];
  double W[729];
  double c;
  double y[351];
  double X_data[351];
  emxArray_real_T *Y;
  int i3;
  emxArray_real_T *b_Y;
  int ar;
  double b_X_data[13];
  double dv3[13];
  int ib;
  int ia;
  int k;
  int cr;
  int ic;
  emxArray_real_T *b;
  int br;

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
  ut_mweights(alpha, beta, kappa, WM, W, &c);
  ut_sigmas(M, P, c, y);
  memcpy(&X_data[0], &y[0], 351U * sizeof(double));
  emxInit_real_T(&Y, 2);

  //
  //  Propagate through the function
  //
  i3 = Y->size[0] * Y->size[1];
  Y->size[0] = 13;
  Y->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)Y, i3, sizeof(double));
  emxInit_real_T(&b_Y, 2);
  for (ar = 0; ar < 27; ar++) {
    memcpy(&b_X_data[0], &X_data[ar * 13], 13U * sizeof(double));
    ukf_track_f_ins(b_X_data, f_param, dv3);
    i3 = b_Y->size[0] * b_Y->size[1];
    b_Y->size[0] = 13;
    b_Y->size[1] = Y->size[1] + 1;
    emxEnsureCapacity((emxArray__common *)b_Y, i3, sizeof(double));
    ib = Y->size[1];
    for (i3 = 0; i3 < ib; i3++) {
      for (ia = 0; ia < 13; ia++) {
        b_Y->data[ia + b_Y->size[0] * i3] = Y->data[ia + Y->size[0] * i3];
      }
    }

    for (i3 = 0; i3 < 13; i3++) {
      b_Y->data[i3 + b_Y->size[0] * Y->size[1]] = dv3[i3];
    }

    i3 = Y->size[0] * Y->size[1];
    Y->size[0] = 13;
    Y->size[1] = b_Y->size[1];
    emxEnsureCapacity((emxArray__common *)Y, i3, sizeof(double));
    ib = b_Y->size[1];
    for (i3 = 0; i3 < ib; i3++) {
      for (ia = 0; ia < 13; ia++) {
        Y->data[ia + Y->size[0] * i3] = b_Y->data[ia + b_Y->size[0] * i3];
      }
    }
  }

  emxFree_real_T(&b_Y);
  if (Y->size[1] == 1) {
    for (i3 = 0; i3 < 13; i3++) {
      M[i3] = 0.0;
      for (ia = 0; ia < 27; ia++) {
        M[i3] += Y->data[i3 + 13 * ia] * WM[ia];
      }
    }
  } else {
    memset(&M[0], 0, 13U * sizeof(double));
    ar = -1;
    for (ib = 0; ib + 1 <= Y->size[1]; ib++) {
      if (WM[ib] != 0.0) {
        ia = ar;
        for (ic = 0; ic < 13; ic++) {
          ia++;
          M[ic] += WM[ib] * Y->data[ia];
        }
      }

      ar += 13;
    }
  }

  if (Y->size[1] == 1) {
    for (i3 = 0; i3 < 13; i3++) {
      for (ia = 0; ia < 27; ia++) {
        y[i3 + 13 * ia] = 0.0;
        for (ar = 0; ar < 27; ar++) {
          y[i3 + 13 * ia] += Y->data[i3 + 13 * ar] * W[ar + 27 * ia];
        }
      }
    }
  } else {
    k = Y->size[1];
    memset(&y[0], 0, 351U * sizeof(double));
    for (cr = 0; cr <= 339; cr += 13) {
      for (ic = cr; ic + 1 <= cr + 13; ic++) {
        y[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= 339; cr += 13) {
      ar = -1;
      i3 = br + k;
      for (ib = br; ib + 1 <= i3; ib++) {
        if (W[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 13; ic++) {
            ia++;
            y[ic] += W[ib] * Y->data[ia];
          }
        }

        ar += 13;
      }

      br += k;
    }
  }

  emxInit_real_T(&b, 2);
  i3 = b->size[0] * b->size[1];
  b->size[0] = Y->size[1];
  b->size[1] = 13;
  emxEnsureCapacity((emxArray__common *)b, i3, sizeof(double));
  for (i3 = 0; i3 < 13; i3++) {
    ib = Y->size[1];
    for (ia = 0; ia < ib; ia++) {
      b->data[ia + b->size[0] * i3] = Y->data[i3 + Y->size[0] * ia];
    }
  }

  if (b->size[0] == 1) {
    for (i3 = 0; i3 < 13; i3++) {
      for (ia = 0; ia < 13; ia++) {
        P[i3 + 13 * ia] = 0.0;
        for (ar = 0; ar < 27; ar++) {
          P[i3 + 13 * ia] += y[i3 + 13 * ar] * b->data[ar + 27 * ia];
        }
      }
    }
  } else {
    memset(&P[0], 0, 169U * sizeof(double));
    if (27 == b->size[0]) {
      for (i3 = 0; i3 < 13; i3++) {
        for (ia = 0; ia < 13; ia++) {
          P[i3 + 13 * ia] = 0.0;
          for (ar = 0; ar < 27; ar++) {
            P[i3 + 13 * ia] += y[i3 + 13 * ar] * b->data[ar + 27 * ia];
          }
        }
      }
    } else {
      for (cr = 0; cr <= 157; cr += 13) {
        for (ic = cr; ic + 1 <= cr + 13; ic++) {
          P[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= 157; cr += 13) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 27; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 13; ic++) {
              ia++;
              P[ic] += b->data[ib] * y[ia];
            }
          }

          ar += 13;
        }

        br += 27;
      }
    }
  }

  memset(&y[0], 0, 351U * sizeof(double));
  for (cr = 0; cr <= 338; cr += 13) {
    for (ic = cr; ic + 1 <= cr + 13; ic++) {
      y[ic] = 0.0;
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
          y[ic] += W[ib] * X_data[ia];
        }
      }

      ar += 13;
    }

    br += 27;
  }

  i3 = b->size[0] * b->size[1];
  b->size[0] = Y->size[1];
  b->size[1] = 13;
  emxEnsureCapacity((emxArray__common *)b, i3, sizeof(double));
  for (i3 = 0; i3 < 13; i3++) {
    ib = Y->size[1];
    for (ia = 0; ia < ib; ia++) {
      b->data[ia + b->size[0] * i3] = Y->data[i3 + Y->size[0] * ia];
    }
  }

  emxFree_real_T(&Y);
  if (b->size[0] == 1) {
    D_size[0] = 13;
    D_size[1] = 13;
    for (i3 = 0; i3 < 13; i3++) {
      for (ia = 0; ia < 13; ia++) {
        D_data[i3 + 13 * ia] = 0.0;
        for (ar = 0; ar < 27; ar++) {
          D_data[i3 + 13 * ia] += y[i3 + 13 * ar] * b->data[ar + 27 * ia];
        }
      }
    }
  } else {
    D_size[0] = 13;
    D_size[1] = 13;
    for (i3 = 0; i3 < 13; i3++) {
      memset(&D_data[i3 * 13], 0, 13U * sizeof(double));
    }

    for (cr = 0; cr <= 156; cr += 13) {
      for (ic = cr; ic + 1 <= cr + 13; ic++) {
        D_data[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= 156; cr += 13) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 27; ib++) {
        if (b->data[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 13; ic++) {
            ia++;
            D_data[ic] += b->data[ib] * y[ia];
          }
        }

        ar += 13;
      }

      br += 27;
    }
  }

  emxFree_real_T(&b);
  for (i3 = 0; i3 < 169; i3++) {
    P[i3] += Q[i3];
  }
}

//
// File trailer for ukf_predict2.cpp
//
// [EOF]
//

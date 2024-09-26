//
// File: ut_mweights.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 12:20:49
//

// Include Files
#include "ukf_predict2.h"
#include "ut_mweights.h"

// Function Definitions

//
// Check which arguments are there
// Arguments    : double alpha
//                double beta
//                double kappa
//                double WM[27]
//                double W[729]
//                double *c
// Return Type  : void
//
void ut_mweights(double alpha, double beta, double kappa, double WM[27], double
                 W[729], double *c)
{
  double lambda;
  int j;
  double wm;
  double wc;
  int ibtile;
  double WC[27];
  double b[729];
  double b_W[729];
  double c_W[729];
  int i0;

  // UT_MWEIGHTS - Generate matrix form unscented transformation weights
  //
  //  Syntax:
  //    [WM,W,c] = ut_mweights(n,alpha,beta,kappa)
  //
  //  In:
  //    n     - Dimensionality of random variable
  //    alpha - Transformation parameter  (optional, default 0.5)
  //    beta  - Transformation parameter  (optional, default 2)
  //    kappa - Transformation parameter  (optional, default 3-size(X,1))
  //
  //  Out:
  //    WM - Weight vector for mean calculation
  //     W - Weight matrix for covariance calculation
  //     c - Scaling constant
  //
  //  Description:
  //    Computes matrix form unscented transformation weights.
  //
  //  Copyright (C) 2006 Simo Särkk&#xFFFD;
  //
  //  $Id: ut_mweights.m 109 2007-09-04 08:32:58Z jmjharti $
  //
  //  This software is distributed under the GNU General Public
  //  Licence (version 2 or later); please refer to the file
  //  Licence.txt, included with the software, for details.
  //
  //  Check which arguments are there
  //
  // UT_WEIGHTS - Generate unscented transformation weights
  //
  //  Syntax:
  //    [WM,WC,c] = ut_weights(n,alpha,beta,kappa)
  //
  //  In:
  //    n     - Dimensionality of random variable
  //    alpha - Transformation parameter  (optional, default 0.5)
  //    beta  - Transformation parameter  (optional, default 2)
  //    kappa - Transformation parameter  (optional, default 3-n)
  //
  //  Out:
  //    WM - Weights for mean calculation
  //    WC - Weights for covariance calculation
  //     c - Scaling constant
  //
  //  Description:
  //    Computes unscented transformation weights.
  //
  //  See also UT_MWEIGHTS UT_TRANSFORM UT_SIGMAS
  //
  //  Copyright (C) 2006 Simo Sï¿½rkkï¿?%
  //  $Id: ut_weights.m 467 2010-10-12 09:30:14Z jmjharti $
  //
  //  This software is distributed under the GNU General Public
  //  Licence (version 2 or later); please refer to the file
  //  Licence.txt, included with the software, for details.
  //
  //  Apply default values
  //
  //
  //  Compute the normal weights
  //
  lambda = alpha * alpha * (13.0 + kappa) - 13.0;
  for (j = 0; j < 27; j++) {
    if (1 + j == 1) {
      wm = lambda / (13.0 + lambda);
      wc = lambda / (13.0 + lambda) + ((1.0 - alpha * alpha) + beta);
    } else {
      wm = 1.0 / (2.0 * (13.0 + lambda));
      wc = wm;
    }

    WM[j] = wm;
    WC[j] = wc;
  }

  *c = 13.0 + lambda;
  memset(&W[0], 0, 729U * sizeof(double));
  for (j = 0; j < 27; j++) {
    W[j + 27 * j] = 1.0;
    ibtile = j * 27;
    memcpy(&b[ibtile], &WM[0], 27U * sizeof(double));
  }

  for (j = 0; j < 729; j++) {
    W[j] -= b[j];
    b[j] = 0.0;
  }

  for (j = 0; j < 27; j++) {
    b[j + 27 * j] = WC[j];
  }

  for (j = 0; j < 27; j++) {
    for (ibtile = 0; ibtile < 27; ibtile++) {
      c_W[j + 27 * ibtile] = 0.0;
      for (i0 = 0; i0 < 27; i0++) {
        c_W[j + 27 * ibtile] += W[j + 27 * i0] * b[i0 + 27 * ibtile];
      }
    }

    for (ibtile = 0; ibtile < 27; ibtile++) {
      b_W[j + 27 * ibtile] = 0.0;
      for (i0 = 0; i0 < 27; i0++) {
        b_W[j + 27 * ibtile] += c_W[j + 27 * i0] * W[ibtile + 27 * i0];
      }
    }
  }

  for (j = 0; j < 27; j++) {
    memcpy(&W[j * 27], &b_W[j * 27], 27U * sizeof(double));
  }
}

//
// File trailer for ut_mweights.cpp
//
// [EOF]
//

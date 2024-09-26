//
// File: ut_sigmas.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 16:03:50
//

// Include Files
#include "ukf_update2.h"
#include "ut_sigmas.h"

// Function Definitions

namespace ukf_update2_ns {
//
// A = schol(P);
// Arguments    : const double M[13]
//                const double P[169]
//                double c
//                double X[351]
// Return Type  : void
//
void ut_sigmas(const double M[13], const double P[169], double c, double X[351])
{
  double A[169];
  int info;
  int colj;
  int j;
  boolean_T exitg1;
  int jj;
  int jmax;
  double ajj;
  int ix;
  int iy;
  int ibtile;
  int i2;
  double b_A[169];
  double b[351];
  double x[351];
  double b_c;
  int ia;

  // UT_SIGMAS - Generate Sigma Points for Unscented Transformation
  //
  //  Syntax:
  //    X = ut_sigmas(M,P,c);
  //
  //  In:
  //    M - Initial state mean (Nx1 column vector)
  //    P - Initial state covariance
  //    c - Parameter returned by UT_WEIGHTS
  //
  //  Out:
  //    X - Matrix where 2N+1 sigma points are as columns
  //
  //  Description:
  //    Generates sigma points and associated weights for Gaussian
  //    initial distribution N(M,P). For default values of parameters
  //    alpha, beta and kappa see UT_WEIGHTS.
  //
  //  See also UT_WEIGHTS UT_TRANSFORM UT_SIGMAS
  //
  //  Copyright (C) 2006 Simo Särkk?%
  //  $Id: ut_sigmas.m 109 2007-09-04 08:32:58Z jmjharti $
  //
  //  This software is distributed under the GNU General Public
  //  Licence (version 2 or later); please refer to the file
  //  Licence.txt, included with the software, for details.
  memcpy(&A[0], &P[0], 169U * sizeof(double));
  info = 0;
  colj = 0;
  j = 1;
  exitg1 = false;
  while ((!exitg1) && (j < 14)) {
    jj = (colj + j) - 1;
    ajj = 0.0;
    if (!(j - 1 < 1)) {
      ix = colj;
      iy = colj;
      for (ibtile = 1; ibtile < j; ibtile++) {
        ajj += A[ix] * A[iy];
        ix++;
        iy++;
      }
    }

    ajj = A[jj] - ajj;
    if (ajj > 0.0) {
      ajj = std::sqrt(ajj);
      A[jj] = ajj;
      if (j < 13) {
        if (j - 1 != 0) {
          iy = jj + 13;
          i2 = (colj + 13 * (12 - j)) + 14;
          for (jmax = colj + 14; jmax <= i2; jmax += 13) {
            ix = colj;
            b_c = 0.0;
            ibtile = (jmax + j) - 2;
            for (ia = jmax; ia <= ibtile; ia++) {
              b_c += A[ia - 1] * A[ix];
              ix++;
            }

            A[iy] += -b_c;
            iy += 13;
          }
        }

        ajj = 1.0 / ajj;
        i2 = (jj + 13 * (12 - j)) + 14;
        for (ibtile = jj + 13; ibtile + 1 <= i2; ibtile += 13) {
          A[ibtile] *= ajj;
        }

        colj += 13;
      }

      j++;
    } else {
      A[jj] = ajj;
      info = j;
      exitg1 = true;
    }
  }

  if (info == 0) {
    jmax = 13;
  } else {
    jmax = info - 1;
  }

  for (j = 0; j + 1 <= jmax; j++) {
    for (ibtile = j + 1; ibtile + 1 <= jmax; ibtile++) {
      A[ibtile + 13 * j] = 0.0;
    }
  }

  for (i2 = 0; i2 < 13; i2++) {
    for (ibtile = 0; ibtile < 13; ibtile++) {
      b_A[ibtile + 13 * i2] = A[i2 + 13 * ibtile];
    }
  }

  ajj = std::sqrt(c);
  for (jmax = 0; jmax < 27; jmax++) {
    ibtile = jmax * 13;
    memcpy(&b[ibtile], &M[0], 13U * sizeof(double));
  }

  for (i2 = 0; i2 < 13; i2++) {
    x[i2] = 0.0;
    for (ibtile = 0; ibtile < 13; ibtile++) {
      x[ibtile + 13 * (i2 + 1)] = ajj * b_A[ibtile + 13 * i2];
      x[ibtile + 13 * (i2 + 14)] = ajj * -b_A[ibtile + 13 * i2];
    }
  }

  for (i2 = 0; i2 < 27; i2++) {
    for (ibtile = 0; ibtile < 13; ibtile++) {
      X[ibtile + 13 * i2] = x[ibtile + 13 * i2] + b[ibtile + 13 * i2];
    }
  }
}

}
//
// File trailer for ut_sigmas.cpp
//
// [EOF]
//

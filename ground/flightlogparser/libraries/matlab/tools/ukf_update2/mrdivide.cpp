//
// File: mrdivide.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 16:03:50
//

// Include Files
#include "ukf_update2.h"
#include "mrdivide.h"

// Function Definitions

//
// Arguments    : double A_data[]
//                int A_size[2]
//                const double B[100]
// Return Type  : void
//
void mrdivide(double A_data[], int A_size[2], const double B[100])
{
  double A[100];
  int k;
  int j;
  signed char ipiv[10];
  int c;
  int kBcol;
  int jp;
  int ix;
  int jAcol;
  double temp;
  int i;
  double s;
  memcpy(&A[0], &B[0], 100U * sizeof(double));
  for (k = 0; k < 10; k++) {
    ipiv[k] = (signed char)(1 + k);
  }

  for (j = 0; j < 9; j++) {
    c = j * 11;
    kBcol = 0;
    ix = c;
    temp = std::abs(A[c]);
    for (k = 2; k <= 10 - j; k++) {
      ix++;
      s = std::abs(A[ix]);
      if (s > temp) {
        kBcol = k - 1;
        temp = s;
      }
    }

    if (A[c + kBcol] != 0.0) {
      if (kBcol != 0) {
        ipiv[j] = (signed char)((j + kBcol) + 1);
        ix = j;
        kBcol += j;
        for (k = 0; k < 10; k++) {
          temp = A[ix];
          A[ix] = A[kBcol];
          A[kBcol] = temp;
          ix += 10;
          kBcol += 10;
        }
      }

      k = (c - j) + 10;
      for (i = c + 1; i + 1 <= k; i++) {
        A[i] /= A[c];
      }
    }

    jp = c;
    jAcol = c + 10;
    for (kBcol = 1; kBcol <= 9 - j; kBcol++) {
      temp = A[jAcol];
      if (A[jAcol] != 0.0) {
        ix = c + 1;
        k = (jp - j) + 20;
        for (i = 11 + jp; i + 1 <= k; i++) {
          A[i] += A[ix] * -temp;
          ix++;
        }
      }

      jAcol += 10;
      jp += 10;
    }
  }

  for (j = 0; j < 10; j++) {
    jp = 13 * j;
    jAcol = 10 * j;
    for (k = 1; k <= j; k++) {
      kBcol = 13 * (k - 1);
      if (A[(k + jAcol) - 1] != 0.0) {
        for (i = 0; i < 13; i++) {
          A_data[i + jp] -= A[(k + jAcol) - 1] * A_data[i + kBcol];
        }
      }
    }

    temp = 1.0 / A[j + jAcol];
    for (i = 0; i < 13; i++) {
      A_data[i + jp] *= temp;
    }
  }

  for (j = 9; j >= 0; j += -1) {
    jp = 13 * j;
    jAcol = 10 * j;
    for (k = j + 1; k + 1 < 11; k++) {
      kBcol = 13 * k;
      if (A[k + jAcol] != 0.0) {
        for (i = 0; i < 13; i++) {
          A_data[i + jp] -= A[k + jAcol] * A_data[i + kBcol];
        }
      }
    }
  }

  for (kBcol = 8; kBcol >= 0; kBcol += -1) {
    if (ipiv[kBcol] != kBcol + 1) {
      jp = ipiv[kBcol] - 1;
      for (jAcol = 0; jAcol < 13; jAcol++) {
        temp = A_data[jAcol + A_size[0] * kBcol];
        A_data[jAcol + A_size[0] * kBcol] = A_data[jAcol + A_size[0] * jp];
        A_data[jAcol + A_size[0] * jp] = temp;
      }
    }
  }
}

//
// File trailer for mrdivide.cpp
//
// [EOF]
//

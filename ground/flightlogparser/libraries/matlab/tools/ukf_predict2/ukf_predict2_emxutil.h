//
// File: ukf_predict2_emxutil.h
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 12:20:49
//
#ifndef UKF_PREDICT2_EMXUTIL_H
#define UKF_PREDICT2_EMXUTIL_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "ukf_predict2_types.h"

// Function Declarations
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, unsigned
  int elementSize);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for ukf_predict2_emxutil.h
//
// [EOF]
//

                                                      //
// File: main.h                                                                                                                                  
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 12:20:49
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
#ifndef _UKF_PREDICT2_MAIN_H
#define _UKF_PREDICT2_MAIN_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "rtwtypes.h"
#include "ukf_predict2_types.h"

// Function Declarations
// extern int main(int argc, const char * const argv[]);
extern void main_ukf_predict2();
// [Xpre, Ppre] = ukf_predict2(X, P, Q, f_param, alpha, beta, kappa, mat);
extern void main_ukf_predict2(std::vector<double>& X,
        std::vector<double>& diagP,
        std::vector<double>& diagQ,
        std::vector<double>& f_param,
        double& alpha,
        double& beta,
        double& kappa,
        double& mat,
        std::vector<double>& Xpre,
        std::vector<std::vector<double>>& Ppre);

#endif

//
// File trailer for main.h
//
// [EOF]
//

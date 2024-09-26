//
// File: main.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 14:58:44
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
// Include Files
#include "ukf_track_h_ins.h"
#include "ukf_track_h_ins_terminate.h"
#include <vector>
#include <iostream>
#include "main.h"

// Function Declarations
static void argInit_13x1_real_T(double result[13]);
static void argInit_1x3_real_T(double result[3]);
static double argInit_real_T();

// Function Definitions

//
// Arguments    : double result[13]
// Return Type  : void
//
static void argInit_13x1_real_T(double result[13])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 13; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_1x3_real_T(double result[3])
{
  int idx1;

  // Loop over the array to initialize each element.
  for (idx1 = 0; idx1 < 3; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

void main_ukf_track_h_ins(std::vector<double>& Xpre,
    std::vector<double>& h_param, std::vector<double>& Zpre) {
  double dv0[13];
  double dv1[3];
  double y[10];
  // Initialize function 'ukf_track_h_ins' input arguments.
  // Initialize function input argument 'x'.
  // Initialize function input argument 'hparam'.
  // Call the entry-point 'ukf_track_h_ins'.
  argInit_13x1_real_T(dv0);
  argInit_1x3_real_T(dv1);

  for (auto i = 0; i < 13; i++) {
      dv0[i] = Xpre[i];
  }
  for (auto i = 0; i < 3; i++) {
      dv1[i] = h_param[i];
  }
  ukf_track_h_ins(dv0, dv1, y);

  Zpre.resize(10);
  for (auto i = 0; i < 10; i++) {
      Zpre[i] = y[i];
  }
  return;
}

//
// Arguments    : void
// Return Type  : void
//
void main_ukf_track_h_ins()
{
  double dv0[13];
  double dv1[3];
  double y[10];
  // Initialize function 'ukf_track_h_ins' input arguments.
  // Initialize function input argument 'x'.
  // Initialize function input argument 'hparam'.
  // Call the entry-point 'ukf_track_h_ins'.
  argInit_13x1_real_T(dv0);
  argInit_1x3_real_T(dv1);

  {
      dv0[0] = -0.0086;
      dv0[1] = -0.0632;
      dv0[2] = 0.1767;
      dv0[3] = 0.8183;
      dv0[4] = -0.1518;
      dv0[5] = -0.1928;
      dv0[6] = 1.0000;
      dv0[7] = -0.0067;
      dv0[8] = 0.0022;
      dv0[9] = -0.0059;
      dv0[10] = 1.5503e-4;
      dv0[11] = -2.2709e-5;
      dv0[12] = 2.9835e-4;
  }
  {
      dv1[0] = 26000;
      dv1[1] = 400;
      dv1[2] = 40000;
  }

  ukf_track_h_ins(dv0, dv1, y);
  for (auto i = 0; i < 10; i++) {
      std::cout << y[i] << " ";
  }
  std::cout << "\n";
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
/*
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_ukf_track_h_ins();

  // Terminate the application.
  // You do not need to do this more than one time.
  ukf_track_h_ins_terminate();
  return 0;
}
*/

//
// File trailer for main.cpp
//
// [EOF]
//

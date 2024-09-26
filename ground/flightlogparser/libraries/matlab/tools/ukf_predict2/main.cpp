//
// File: main.cpp
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
// Include Files
#include "ukf_predict2.h"
#include "main.h"
#include "ukf_predict2_terminate.h"
#include <iostream> 
#include <vector>

// Function Declarations
static void argInit_13x13_real_T(double result[169]);
static void argInit_13x1_real_T(double result[13]);
static void argInit_1x7_real_T(double result[7]);
static double argInit_real_T();

// Function Definitions

//
// Arguments    : double result[169]
// Return Type  : void
//
static void argInit_13x13_real_T(double result[169])
{
  int idx0;
  int idx1;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 13; idx0++) {
    for (idx1 = 0; idx1 < 13; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 13 * idx1] = argInit_real_T();
    }
  }
}

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
// Arguments    : double result[7]
// Return Type  : void
//
static void argInit_1x7_real_T(double result[7])
{
  int idx1;

  // Loop over the array to initialize each element.
  for (idx1 = 0; idx1 < 7; idx1++) {
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

void main_ukf_predict2(std::vector<double>& M,
        std::vector<double>& diagP,
        std::vector<double>& diagQ,
        std::vector<double>& f_param,
        double& alpha,
        double& beta,
        double& kappa,
        double& mat,
        std::vector<double>& Xpre,
        std::vector<std::vector<double>>& Ppre) {
  double X[13];
  double P[169];
  double dv1[169];
  double dv2[7];
  double D_data[169];
  int D_size[2];

  // Initialize function 'ukf_predict2' input arguments.
  // Initialize function input argument 'M'.
  argInit_13x1_real_T(X);

  // Initialize function input argument 'P'.
  argInit_13x13_real_T(P);

  // init values
  const size_t NUM_STATE = 13;
  for (auto i = 0; i < NUM_STATE; i++) {
      for (auto j = 0; j < NUM_STATE; j++) {
          if (i == j) {
              P[i * NUM_STATE + j] = diagP[i];
          } else {
              P[i * NUM_STATE + j] = 0;
          }
      }
  }
  auto Q = P;

  // Initialize function input argument 'Q'.
  // Initialize function input argument 'f_param'.
  // Call the entry-point 'ukf_predict2'.
  argInit_13x13_real_T(dv1);
  argInit_1x7_real_T(dv2);

  const size_t F_PARAM_SIZE = 7;
  for (auto i = 0; i < F_PARAM_SIZE; i++) {
      dv2[i] = f_param[i];
  }

  for (auto i = 0; i < NUM_STATE; i++) {
      X[i] = M[i];
  }

  ukf_predict2(X, P, Q, dv2, alpha, beta,
               kappa, mat, D_data, D_size);

  for (auto i = 0; i < NUM_STATE; i++) {
      Ppre.push_back({});
      Ppre[i].resize(NUM_STATE);
      for (auto j = 0; j < NUM_STATE; j++) {
          Ppre[i][j] = P[i * NUM_STATE + j];
      }
  }
  Xpre.resize(NUM_STATE);
  for (auto i = 0; i < NUM_STATE; i++) {
      Xpre[i] = X[i];
  }
}

//
// Arguments    : void
// Return Type  : void
//
void main_ukf_predict2()
{
  double X[13];
  double P[169];
  double dv1[169];
  double dv2[7];
  double D_data[169];
  int D_size[2];

  // Initialize function 'ukf_predict2' input arguments.
  // Initialize function input argument 'M'.
  argInit_13x1_real_T(X);

  // Initialize function input argument 'P'.
  argInit_13x13_real_T(P);

  // init values
  const size_t NUM_STATE = 13;
  std::vector<double> diagP = {25.0, 25.0, 25.0, 5.0, 5.0, 5.0,
    1e-4, 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6};
  for (auto i = 0; i < NUM_STATE; i++) {
      for (auto j = 0; j < NUM_STATE; j++) {
          if (i == j) {
              P[i * NUM_STATE + j] = diagP[i];
          } else {
              P[i * NUM_STATE + j] = 0;
          }
      }
  }
  auto Q = P;
  /*
  R = diag([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 10, ...
      10, 10, 0.01]);
  */
  double alpha = 0.1;
  double beta = 2;
  double kappa = 0;
  double mat = 1;

  // Initialize function input argument 'Q'.
  // Initialize function input argument 'f_param'.
  // Call the entry-point 'ukf_predict2'.
  argInit_13x13_real_T(dv1);
  argInit_1x7_real_T(dv2);

  {
      dv2[0] = 0.0053;
      dv2[1] = -0.4103;
      dv2[2] = -1.4706;
      dv2[3] = 0.8612;
      dv2[4] = 0.1112;
      dv2[5] = 0.0608;
      dv2[6] = -9.8812;
  }
  {
      X[0] = -0.0129;
      X[1] = -0.0624;
      X[2] = -0.1777;
      X[3] = 0.8182;
      X[4] = -0.1515;
      X[5] = -0.1924;
      X[6] = -0.9999;
      X[7] = -0.0056;
      X[8] = -0.0060;
      X[9] = -0.0082;
      X[10] = 1.5503e-04;
      X[11] = -2.2709e-05;
      X[12] = 2.9835e-04;
  }
  ukf_predict2(X, P, Q, dv2, alpha, beta,
               kappa, mat, D_data, D_size);
  /*
  for (auto i = 0; i < NUM_STATE; i++) {
      for (auto j = 0; j < NUM_STATE; j++) {
          std::cout << D_data[i * NUM_STATE + j] << " ";
      }
      std::cout << std::endl;
  }
  */

  // real output P X 
  std::cout << "printing P matrix\n";
  for (auto i = 0; i < NUM_STATE; i++) {
      for (auto j = 0; j < NUM_STATE; j++) {
          std::cout << P[i * NUM_STATE + j] << " ";
      }
      std::cout << std::endl;
  }

  // D_size
  // std::cout << D_size[0] << " " << D_size[1] << std::endl;
  // 
  std::cout << "printing X matrix\n";
  for (auto i = 0; i < NUM_STATE; i++) {
      std::cout << X[i] << " ";
  }
  std::cout << std::endl;
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
  main_ukf_predict2();

  // Terminate the application.
  // You do not need to do this more than one time.
  ukf_predict2_terminate();
  std::cout << "test pass\n";
  return 0;
}
*/

//
// File trailer for main.cpp
//
// [EOF]
//

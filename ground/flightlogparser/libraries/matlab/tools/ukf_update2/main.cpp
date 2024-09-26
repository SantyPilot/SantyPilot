//
// File: main.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 07-Jul-2024 16:03:50
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
#include "ukf_update2.h"
#include "ukf_update2_terminate.h"
#include <iostream>
#include <vector>

// Function Declarations
static void argInit_10x10_real_T(double result[100]);
static void argInit_10x1_real_T(double result[10]);
static void argInit_13x13_real_T(double result[169]);
static void argInit_13x1_real_T(double result[13]);
static void argInit_1x3_real_T(double result[3]);
static double argInit_real_T();

// Function Definitions

//
// Arguments    : double result[100]
// Return Type  : void
//
static void argInit_10x10_real_T(double result[100])
{
  int idx0;
  int idx1;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 10; idx0++) {
    for (idx1 = 0; idx1 < 10; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 10 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result[10]
// Return Type  : void
//
static void argInit_10x1_real_T(double result[10])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 10; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

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

extern void main_ukf_update2(std::vector<double>& Xpre,
    std::vector<std::vector<double>>& Ppre,
    std::vector<double>& Z,
    std::vector<double>& Rdiag,
    std::vector<double>& h_param,
    double alpha,
    double beta,
    double kappa,
    double mat,
    std::vector<double>& X,
    std::vector<std::vector<double>>& Pcor) {
  double M[13];
  double P[169];
  double dv1[10];
  double R[100];
  double dv3[3];
  double K_data[130];
  int K_size[2];

  const size_t NUM_STATE = 13;
  // Initialize function 'ukf_update2' input arguments.
  // Initialize function input argument 'M'.
  argInit_13x1_real_T(M);

  // Initialize function input argument 'P'.
  argInit_13x13_real_T(P);

  for (auto i = 0; i < NUM_STATE; i++) {
      for (auto j = 0; j < NUM_STATE; j++) {
          P[i * NUM_STATE + j] = Ppre[i][j];
      }
  }
  for (auto i = 0; i < NUM_STATE; i++) {
      M[i] = Xpre[i];
  }
  // Initialize function input argument 'Y'.
  // Initialize function input argument 'R'.
  // Initialize function input argument 'h_param'.
  // Call the entry-point 'ukf_update2'.
  argInit_10x1_real_T(dv1);
  argInit_10x10_real_T(R);
  argInit_1x3_real_T(dv3);

  for (auto i = 0; i < 10; i++) {
      dv1[i] = Z[i];
  }
  const size_t NUM_OBS = 10;
  for (auto i = 0; i < NUM_OBS; i++) {
      for (auto j = 0; j < NUM_OBS; j++) {
          if (i == j) {
              R[i * NUM_OBS + j] = Rdiag[i];
          } else {
              R[i * NUM_OBS + j] = 0;
          }
      }
  }
  for (auto i = 0; i < 3; i++) {
      dv3[i] = h_param[i];
  }
  ukf_update2(M, P, dv1, R, dv3, alpha, beta, 
              kappa, mat, K_data, K_size);

  X.resize(NUM_STATE);
  for (auto i = 0; i < NUM_STATE; i++) {
      X[i] = M[i];
  }

  for (auto i = 0; i < NUM_STATE; i++) {
      Pcor.push_back({});
      Pcor[i].resize(NUM_STATE);
      for (auto j = 0; j < NUM_STATE; j++) {
          Pcor[i][j] = K_data[i * NUM_STATE + j];
      }
  }
}

//
// Arguments    : void
// Return Type  : void
//
void main_ukf_update2()
{
  double M[13];
  double P[169];
  double dv1[10];
  double R[100];
  double dv3[3];
  double K_data[130];
  int K_size[2];

  // Initialize function 'ukf_update2' input arguments.
  // Initialize function input argument 'M'.
  argInit_13x1_real_T(M);

  // Initialize function input argument 'P'.
  argInit_13x13_real_T(P);

  const size_t NUM_STATE = 13;
  std::vector<double> diagP = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0,
    1e-3, 2e-4, 2e-4, 2e-4, 2e-6, 2e-6, 2e-6};
  for (auto i = 0; i < NUM_STATE; i++) {
      for (auto j = 0; j < NUM_STATE; j++) {
          if (i == j) {
              P[i * NUM_STATE + j] = diagP[i];
          } else {
              P[i * NUM_STATE + j] = 0;
          }
      }
  }

  {
      M[0] = -0.0086;
      M[1] = -0.0632;
      M[2] = 0.1767;
      M[3] = 0.8183;
      M[4] = -0.1518;
      M[5] = -0.1928;
      M[6] = 1.0000;
      M[7] = -0.0067;
      M[8] = 0.0022;
      M[9] = -0.0059;
      M[10] = 1.5503e-4;
      M[11] = -2.2709e-5;
      M[12] = 2.9835e-4;
  }


  // Initialize function input argument 'Y'.
  // Initialize function input argument 'R'.
  // Initialize function input argument 'h_param'.
  // Call the entry-point 'ukf_update2'.
  argInit_10x1_real_T(dv1);
  argInit_10x10_real_T(R);
  argInit_1x3_real_T(dv3);

  {
      dv1[0] = -0.0675;
      dv1[1] = 0.2134;
      dv1[2] = -0.1114;
      dv1[3] = 0.1083;
      dv1[4] = 0.6788;
      dv1[5] = -0.0561;
      dv1[6] = 26000;
      dv1[7] = 400;
      dv1[8] = 40000;
      dv1[9] = 0;
  }
  const size_t NUM_OBS = 10;
  std::vector<double> diagR = {1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3,
    10, 10, 10, 0.01};
  for (auto i = 0; i < NUM_OBS; i++) {
      for (auto j = 0; j < NUM_OBS; j++) {
          if (i == j) {
              R[i * NUM_OBS + j] = diagR[i];
          } else {
              R[i * NUM_OBS + j] = 0;
          }
      }
  }
  {
      dv3[0] = 26000;
      dv3[1] = 400;
      dv3[2] = 40000;
  }
  
  double alpha = 0.1;
  double beta = 2;
  double kappa = 0;
  double mat = 1;

  // real output P X 
  std::cout << "before update printing P matrix\n";
  for (auto i = 0; i < NUM_STATE; i++) {
      for (auto j = 0; j < NUM_STATE; j++) {
          std::cout << P[i * NUM_STATE + j] << " ";
      }
      std::cout << std::endl;
  }
  ukf_update2(M, P, dv1, R, dv3, alpha, beta, 
              kappa, mat, K_data, K_size);
  // print result
  // real output P X 
  std::cout << "printing X matrix\n";
  for (auto i = 0; i < NUM_STATE; i++) {
      std::cout << M[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "printing P matrix\n";
  for (auto i = 0; i < NUM_STATE; i++) {
      for (auto j = 0; j < NUM_STATE; j++) {
          std::cout << K_data[i * NUM_STATE + j] << " ";
      }
      std::cout << std::endl;
  }

  // K_size
  std::cout << K_size[0] << " " << K_size[1] << std::endl;
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
  main_ukf_update2();

  // Terminate the application.
  // You do not need to do this more than one time.
  ukf_update2_terminate();
  return 0;
}
*/

//
// File trailer for main.cpp
//
// [EOF]
//

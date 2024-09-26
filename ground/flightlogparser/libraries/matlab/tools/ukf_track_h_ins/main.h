#ifndef _UKF_TRACK_H_INS_MAIN_H
#define _UKF_TRACK_H_INS_MAIN_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "rtwtypes.h"

// Function Declarations
// extern int main(int argc, const char * const argv[]);
// extern void main_ukf_track_h_ins();
// Zpre = ukf_track_h_ins(Xpre, hparam);
extern void main_ukf_track_h_ins(std::vector<double>& Xpre,
    std::vector<double>& h_param, std::vector<double>& Zpre);


#endif


#ifndef _MY_ALGO_HR_H
#define _MY_ALGO_HR_H

#include <cstdint>
#include <math.h>

using namespace std;

int16_t HRFunc(int32_t sample);

int32_t bp_fir_filter(int32_t din); // Bandpass 2nd order FIR filter 

#endif

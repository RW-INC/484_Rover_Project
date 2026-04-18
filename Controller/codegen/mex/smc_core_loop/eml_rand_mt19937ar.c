/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eml_rand_mt19937ar.c
 *
 * Code generation for function 'eml_rand_mt19937ar'
 *
 */

/* Include files */
#include "eml_rand_mt19937ar.h"
#include "rt_nonfinite.h"
#include "smc_core_loop.h"
#include "omp.h"

/* Function Definitions */
void eml_rand_mt19937ar(emlrtCTX aTLS, uint32_T c_state[625])
{
  int32_T mti;
  if (emlrtIsInParallelRegion(aTLS)) {
    int32_T q0;
    uint32_T r;
    q0 = getThreadID();
    if (q0 > 2147483646) {
      q0 = MAX_int32_T;
    } else {
      q0++;
    }
    if (q0 < 0) {
      q0 = 0;
    }
    r = (uint32_T)q0;
    c_state[0] = (uint32_T)q0;
    for (mti = 0; mti < 623; mti++) {
      r = ((r ^ r >> 30U) * 1812433253U + (uint32_T)mti) + 1U;
      c_state[mti + 1] = r;
    }
    c_state[624] = 624U;
  } else {
    uint32_T r;
    r = 5489U;
    c_state[0] = 5489U;
    for (mti = 0; mti < 623; mti++) {
      r = ((r ^ r >> 30U) * 1812433253U + (uint32_T)mti) + 1U;
      c_state[mti + 1] = r;
    }
    c_state[624] = 624U;
  }
}

void genrand_uint32_vector(uint32_T mt[625], uint32_T u[2])
{
  int32_T j;
  int32_T kk;
  for (j = 0; j < 2; j++) {
    uint32_T mti;
    uint32_T y;
    mti = mt[624] + 1U;
    if (mti >= 625U) {
      for (kk = 0; kk < 227; kk++) {
        mti = (mt[kk] & 2147483648U) | (mt[kk + 1] & 2147483647U);
        if ((mti & 1U) == 0U) {
          mti >>= 1U;
        } else {
          mti = mti >> 1U ^ 2567483615U;
        }
        mt[kk] = mt[kk + 397] ^ mti;
      }
      for (kk = 0; kk < 396; kk++) {
        mti = (mt[kk + 227] & 2147483648U) | (mt[kk + 228] & 2147483647U);
        if ((mti & 1U) == 0U) {
          mti >>= 1U;
        } else {
          mti = mti >> 1U ^ 2567483615U;
        }
        mt[kk + 227] = mt[kk] ^ mti;
      }
      mti = (mt[623] & 2147483648U) | (mt[0] & 2147483647U);
      if ((mti & 1U) == 0U) {
        mti >>= 1U;
      } else {
        mti = mti >> 1U ^ 2567483615U;
      }
      mt[623] = mt[396] ^ mti;
      mti = 1U;
    }
    y = mt[(int32_T)mti - 1];
    mt[624] = mti;
    y ^= y >> 11U;
    y ^= y << 7U & 2636928640U;
    y ^= y << 15U & 4022730752U;
    u[j] = y ^ y >> 18U;
  }
}

/* End of code generation (eml_rand_mt19937ar.c) */

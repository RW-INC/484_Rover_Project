/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * smc_core_loop_emxutil.h
 *
 * Code generation for function 'smc_core_loop_emxutil'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "smc_core_loop_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void emxEnsureCapacity_real32_T(emxArray_real32_T *emxArray, int32_T oldNumel);

void emxFree_real32_T(emlrtCTX aTLS, emxArray_real32_T **pEmxArray);

void emxInit_real32_T(emlrtCTX aTLS, emxArray_real32_T **pEmxArray,
                      int32_T numDimensions);

/* End of code generation (smc_core_loop_emxutil.h) */

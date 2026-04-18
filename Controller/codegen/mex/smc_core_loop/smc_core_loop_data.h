/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * smc_core_loop_data.h
 *
 * Code generation for function 'smc_core_loop_data'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include "omp.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern boolean_T state_not_empty_main;
extern boolean_T method_not_empty_main;
extern boolean_T b_method_not_empty_main;
extern boolean_T b_state_not_empty_main;
extern emlrtContext emlrtContextGlobal;
extern omp_lock_t emlrtLockGlobal;
extern omp_nest_lock_t smc_core_loop_nestLockGlobal;

/* End of code generation (smc_core_loop_data.h) */

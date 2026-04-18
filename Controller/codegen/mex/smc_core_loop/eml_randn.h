/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eml_randn.h
 *
 * Code generation for function 'eml_randn'
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
void eml_randn(real_T varargin_2, emxArray_real32_T *r);

void eml_randn_init(emlrtCTX aTLS);

void eml_randn_swap(boolean_T aToMain);

/* End of code generation (eml_randn.h) */

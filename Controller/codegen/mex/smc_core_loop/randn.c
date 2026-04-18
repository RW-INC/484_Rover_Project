/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * randn.c
 *
 * Code generation for function 'randn'
 *
 */

/* Include files */
#include "randn.h"
#include "eml_randn.h"
#include "rt_nonfinite.h"
#include "smc_core_loop_types.h"

/* Function Definitions */
void randn(real_T varargin_2, emxArray_real32_T *r)
{
  eml_randn(varargin_2, r);
}

/* End of code generation (randn.c) */

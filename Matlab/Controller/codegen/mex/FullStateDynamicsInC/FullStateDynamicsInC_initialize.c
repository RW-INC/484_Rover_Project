/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * FullStateDynamicsInC_initialize.c
 *
 * Code generation for function 'FullStateDynamicsInC_initialize'
 *
 */

/* Include files */
#include "FullStateDynamicsInC_initialize.h"
#include "FullStateDynamicsInC_data.h"
#include "_coder_FullStateDynamicsInC_mex.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void FullStateDynamicsInC_once(void);

/* Function Definitions */
static void FullStateDynamicsInC_once(void)
{
  mex_InitInfAndNan();
}

void FullStateDynamicsInC_initialize(void)
{
  static const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    FullStateDynamicsInC_once();
  }
}

/* End of code generation (FullStateDynamicsInC_initialize.c) */

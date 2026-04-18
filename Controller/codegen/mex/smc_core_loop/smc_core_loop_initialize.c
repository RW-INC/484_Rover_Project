/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * smc_core_loop_initialize.c
 *
 * Code generation for function 'smc_core_loop_initialize'
 *
 */

/* Include files */
#include "smc_core_loop_initialize.h"
#include "_coder_smc_core_loop_mex.h"
#include "eml_rand.h"
#include "eml_rand_mcg16807_stateful.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "eml_rand_shr3cong_stateful.h"
#include "eml_randn.h"
#include "rt_nonfinite.h"
#include "smc_core_loop.h"
#include "smc_core_loop_data.h"

/* Function Declarations */
static void smc_core_loop_once(void);

/* Function Definitions */
static void smc_core_loop_once(void)
{
  mex_InitInfAndNan();
  b_state_not_empty_main = false;
  b_method_not_empty_main = false;
  method_not_empty_main = false;
  state_not_empty_main = false;
  initThreadID(emlrtRootTLSGlobal);
  eml_randn_init(emlrtRootTLSGlobal);
  eml_rand_init(emlrtRootTLSGlobal);
  eml_rand_mcg16807_stateful_init();
  eml_rand_shr3cong_stateful_init();
  c_eml_rand_mt19937ar_stateful_i(emlrtRootTLSGlobal);
}

void smc_core_loop_initialize(void)
{
  mexFunctionCreateRootTLS();
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, NULL);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    smc_core_loop_once();
  }
}

/* End of code generation (smc_core_loop_initialize.c) */

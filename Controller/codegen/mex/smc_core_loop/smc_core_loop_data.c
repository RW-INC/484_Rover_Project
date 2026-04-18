/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * smc_core_loop_data.c
 *
 * Code generation for function 'smc_core_loop_data'
 *
 */

/* Include files */
#include "smc_core_loop_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

boolean_T state_not_empty_main;

boolean_T method_not_empty_main;

boolean_T b_method_not_empty_main;

boolean_T b_state_not_empty_main;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131675U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "smc_core_loop",                                      /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

omp_lock_t emlrtLockGlobal;

omp_nest_lock_t smc_core_loop_nestLockGlobal;

/* End of code generation (smc_core_loop_data.c) */

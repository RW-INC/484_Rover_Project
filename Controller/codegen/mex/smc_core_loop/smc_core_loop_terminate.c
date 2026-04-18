/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * smc_core_loop_terminate.c
 *
 * Code generation for function 'smc_core_loop_terminate'
 *
 */

/* Include files */
#include "smc_core_loop_terminate.h"
#include "_coder_smc_core_loop_mex.h"
#include "rt_nonfinite.h"
#include "smc_core_loop_data.h"
#include "omp.h"

/* Function Declarations */
static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void smc_core_loop_atexit(void)
{
  static jmp_buf emlrtJBEnviron;
  mexFunctionCreateRootTLS();
  emlrtLoadLibrary("C:\\ProgramData\\MATLAB\\SupportPackages\\R2025b\\3P."
                   "instrset\\mingw_w64.instrset\\bin\\libgomp-1.dll");
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&smc_core_loop_nestLockGlobal);
  emlrtSetJmpBuf(emlrtRootTLSGlobal, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    emlrtPushHeapReferenceStackR2021a(emlrtRootTLSGlobal, false, NULL,
                                      (void *)&emlrtExitTimeCleanupDtorFcn,
                                      NULL, NULL, NULL);
    emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
    emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
    emlrtExitTimeCleanup(&emlrtContextGlobal);
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&smc_core_loop_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&smc_core_loop_nestLockGlobal);
    emlrtReportParallelRunTimeError(emlrtRootTLSGlobal);
  }
}

void smc_core_loop_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (smc_core_loop_terminate.c) */

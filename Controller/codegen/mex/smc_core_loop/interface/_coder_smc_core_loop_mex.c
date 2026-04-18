/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_smc_core_loop_mex.c
 *
 * Code generation for function '_coder_smc_core_loop_mex'
 *
 */

/* Include files */
#include "_coder_smc_core_loop_mex.h"
#include "_coder_smc_core_loop_api.h"
#include "rt_nonfinite.h"
#include "smc_core_loop.h"
#include "smc_core_loop_data.h"
#include "smc_core_loop_initialize.h"
#include "smc_core_loop_terminate.h"
#include "omp.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  static jmp_buf emlrtJBEnviron;
  mexAtExit(&smc_core_loop_atexit);
  emlrtLoadLibrary("C:\\ProgramData\\MATLAB\\SupportPackages\\R2025b\\3P."
                   "instrset\\mingw_w64.instrset\\bin\\libgomp-1.dll");
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&smc_core_loop_nestLockGlobal);
  smc_core_loop_initialize();
  emlrtSetJmpBuf(emlrtRootTLSGlobal, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    unsafe_smc_core_loop_mexFunction(nlhs, plhs, nrhs, prhs);
    smc_core_loop_terminate();
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&smc_core_loop_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&smc_core_loop_nestLockGlobal);
    emlrtReportParallelRunTimeError(emlrtRootTLSGlobal);
  }
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal,
                           &emlrtLockerFunction, omp_get_num_procs(), NULL,
                           "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void unsafe_smc_core_loop_mexFunction(int32_T nlhs, mxArray *plhs[6],
                                      int32_T nrhs, const mxArray *prhs[21])
{
  const mxArray *outputs[6];
  int32_T i;
  /* Check for proper number of arguments. */
  if (nrhs != 21) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal, "EMLRT:runTime:WrongNumberOfInputs",
                        5, 12, 21, 4, 13, "smc_core_loop");
  }
  if (nlhs > 6) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal,
                        "EMLRT:runTime:TooManyOutputArguments", 3, 4, 13,
                        "smc_core_loop");
  }
  /* Call the function. */
  smc_core_loop_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

/* End of code generation (_coder_smc_core_loop_mex.c) */

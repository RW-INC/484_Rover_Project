/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_FullStateDynamicsInC_mex.c
 *
 * Code generation for function '_coder_FullStateDynamicsInC_mex'
 *
 */

/* Include files */
#include "_coder_FullStateDynamicsInC_mex.h"
#include "FullStateDynamicsInC.h"
#include "FullStateDynamicsInC_data.h"
#include "FullStateDynamicsInC_initialize.h"
#include "FullStateDynamicsInC_terminate.h"
#include "FullStateDynamicsInC_types.h"
#include "_coder_FullStateDynamicsInC_api.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Function Definitions */
void FullStateDynamicsInC_mexFunction(FullStateDynamicsInCStackData *SD,
                                      int32_T nlhs, mxArray *plhs[1],
                                      int32_T nrhs, const mxArray *prhs[10])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 10) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 10, 4,
                        20, "FullStateDynamicsInC");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 20,
                        "FullStateDynamicsInC");
  }
  /* Call the function. */
  FullStateDynamicsInC_api(SD, prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  static jmp_buf emlrtJBEnviron;
  FullStateDynamicsInCStackData *c_FullStateDynamicsInCStackData = NULL;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  c_FullStateDynamicsInCStackData =
      (FullStateDynamicsInCStackData *)emlrtMxCalloc(
          (size_t)1, (size_t)1U * sizeof(FullStateDynamicsInCStackData));
  mexAtExit(&FullStateDynamicsInC_atexit);
  emlrtLoadLibrary("C:\\ProgramData\\MATLAB\\SupportPackages\\R2025b\\3P."
                   "instrset\\mingw_w64.instrset\\bin\\libgomp-1.dll");
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&FullStateDynamicsInC_nestLockGlobal);
  FullStateDynamicsInC_initialize();
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    FullStateDynamicsInC_mexFunction(c_FullStateDynamicsInCStackData, nlhs,
                                     plhs, nrhs, prhs);
    FullStateDynamicsInC_terminate();
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&FullStateDynamicsInC_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&FullStateDynamicsInC_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
  emlrtMxFree(c_FullStateDynamicsInCStackData);
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal,
                           &emlrtLockerFunction, omp_get_num_procs(), NULL,
                           "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_FullStateDynamicsInC_mex.c) */

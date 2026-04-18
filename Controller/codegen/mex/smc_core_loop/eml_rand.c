/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eml_rand.c
 *
 * Code generation for function 'eml_rand'
 *
 */

/* Include files */
#include "eml_rand.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "rt_nonfinite.h"
#include "smc_core_loop_data.h"
#include "smc_core_loop_types.h"
#include "omp.h"

/* Variable Definitions */
static uint32_T b_method;
#pragma omp threadprivate(b_method)

static boolean_T b_method_not_empty;
#pragma omp threadprivate(b_method_not_empty)

static uint32_T method_main;

/* Function Definitions */
void eml_rand(real_T varargin_2, emxArray_real32_T *r)
{
  eml_rand_mt19937ar_stateful(varargin_2, r);
}

void eml_rand_init(emlrtCTX aTLS)
{
  jmp_buf *volatile emlrtJBStack;
  int32_T eml_rand_init_numThreads;
  int32_T i;
  int32_T ub_loop;
  ub_loop = omp_get_max_threads();
  emlrtEnterParallelRegion(aTLS, omp_in_parallel());
  emlrtPushJmpBuf(aTLS, &emlrtJBStack);
  eml_rand_init_numThreads = emlrtAllocRegionTLSs(
      aTLS, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for schedule(static) num_threads(eml_rand_init_numThreads)

  for (i = 1; i <= ub_loop; i++) {
    b_method = 7U;
    b_method_not_empty = true;
  }
  emlrtPopJmpBuf(aTLS, &emlrtJBStack);
  emlrtExitParallelRegion(aTLS, omp_in_parallel());
  method_main = 7U;
  method_not_empty_main = true;
}

void eml_rand_swap(void)
{
  uint32_T method_tmp;
  boolean_T method_not_empty_tmp;
  method_tmp = b_method;
  b_method = method_main;
  method_main = method_tmp;
  method_not_empty_tmp = b_method_not_empty;
  b_method_not_empty = method_not_empty_main;
  method_not_empty_main = method_not_empty_tmp;
}

/* End of code generation (eml_rand.c) */

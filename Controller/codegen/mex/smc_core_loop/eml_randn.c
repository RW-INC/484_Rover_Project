/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eml_randn.c
 *
 * Code generation for function 'eml_randn'
 *
 */

/* Include files */
#include "eml_randn.h"
#include "eml_rand.h"
#include "rt_nonfinite.h"
#include "smc_core_loop_data.h"
#include "smc_core_loop_types.h"
#include "omp.h"

/* Variable Definitions */
static uint32_T method;
#pragma omp threadprivate(method)

static boolean_T method_not_empty;
#pragma omp threadprivate(method_not_empty)

static uint32_T state[2];
#pragma omp threadprivate(state)

static boolean_T state_not_empty;
#pragma omp threadprivate(state_not_empty)

static uint32_T b_method_main;

static uint32_T b_state_main[2];

static uint32_T (*b_state_ptr)[2];
#pragma omp threadprivate(b_state_ptr)

/* Function Definitions */
void eml_randn(real_T varargin_2, emxArray_real32_T *r)
{
  eml_rand(varargin_2, r);
}

void eml_randn_init(emlrtCTX aTLS)
{
  jmp_buf *volatile emlrtJBStack;
  int32_T eml_randn_init_numThreads;
  int32_T i;
  int32_T ub_loop;
  ub_loop = omp_get_max_threads();
  emlrtEnterParallelRegion(aTLS, omp_in_parallel());
  emlrtPushJmpBuf(aTLS, &emlrtJBStack);
  eml_randn_init_numThreads = emlrtAllocRegionTLSs(
      aTLS, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for schedule(static) num_threads(eml_randn_init_numThreads)

  for (i = 1; i <= ub_loop; i++) {
    state_not_empty = false;
    method_not_empty = false;
    b_state_ptr = (uint32_T(*)[2])state;
    method = 0U;
    method_not_empty = true;
    state[0] = 362436069U;
    state[1] = 521288629U;
    state_not_empty = true;
  }
  emlrtPopJmpBuf(aTLS, &emlrtJBStack);
  emlrtExitParallelRegion(aTLS, omp_in_parallel());
  b_method_main = 0U;
  b_method_not_empty_main = true;
  b_state_main[0] = 362436069U;
  b_state_main[1] = 521288629U;
  b_state_not_empty_main = true;
}

void eml_randn_swap(boolean_T aToMain)
{
  uint32_T method_tmp;
  boolean_T method_not_empty_tmp;
  method_tmp = method;
  method = b_method_main;
  b_method_main = method_tmp;
  method_not_empty_tmp = method_not_empty;
  method_not_empty = b_method_not_empty_main;
  b_method_not_empty_main = method_not_empty_tmp;
  if (aToMain) {
    b_state_ptr = (uint32_T(*)[2])b_state_main;
  } else {
    b_state_ptr = (uint32_T(*)[2])state;
  }
  method_not_empty_tmp = state_not_empty;
  state_not_empty = b_state_not_empty_main;
  b_state_not_empty_main = method_not_empty_tmp;
}

/* End of code generation (eml_randn.c) */

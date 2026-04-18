/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eml_rand_shr3cong_stateful.c
 *
 * Code generation for function 'eml_rand_shr3cong_stateful'
 *
 */

/* Include files */
#include "eml_rand_shr3cong_stateful.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void eml_rand_shr3cong_stateful_init(void)
{
  static uint32_T c_state[2];
  static boolean_T c_state_not_empty;
  c_state[0] = 362436069U;
  c_state[1] = 521288629U;
  c_state_not_empty = true;
}

/* End of code generation (eml_rand_shr3cong_stateful.c) */

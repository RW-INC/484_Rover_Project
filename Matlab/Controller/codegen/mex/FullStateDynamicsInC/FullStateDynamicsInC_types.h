/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * FullStateDynamicsInC_types.h
 *
 * Code generation for function 'FullStateDynamicsInC'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_struct0_T
#define typedef_struct0_T
typedef struct {
  real_T r;
  real_T B;
  real_T L;
  real_T max_w;
} struct0_T;
#endif /* typedef_struct0_T */

#ifndef typedef_b_captured_var
#define typedef_b_captured_var
typedef struct {
  real_T contents[1000000];
} b_captured_var;
#endif /* typedef_b_captured_var */

#ifndef typedef_b_FullStateDynamicsInC
#define typedef_b_FullStateDynamicsInC
typedef struct {
  b_captured_var Z_map;
} b_FullStateDynamicsInC;
#endif /* typedef_b_FullStateDynamicsInC */

#ifndef c_typedef_FullStateDynamicsInCS
#define c_typedef_FullStateDynamicsInCS
typedef struct {
  b_FullStateDynamicsInC f0;
} FullStateDynamicsInCStackData;
#endif /* c_typedef_FullStateDynamicsInCS */

/* End of code generation (FullStateDynamicsInC_types.h) */

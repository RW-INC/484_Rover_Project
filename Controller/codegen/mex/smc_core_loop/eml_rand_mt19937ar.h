/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eml_rand_mt19937ar.h
 *
 * Code generation for function 'eml_rand_mt19937ar'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void eml_rand_mt19937ar(emlrtCTX aTLS, uint32_T c_state[625]);

void genrand_uint32_vector(uint32_T mt[625], uint32_T u[2]);

/* End of code generation (eml_rand_mt19937ar.h) */

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * FullStateDynamicsInC.h
 *
 * Code generation for function 'FullStateDynamicsInC'
 *
 */

#pragma once

/* Include files */
#include "FullStateDynamicsInC_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void FullStateDynamicsInC(FullStateDynamicsInCStackData *SD,
                          const emlrtStack *sp, const real_T state[9],
                          const real_T u[4], real_T xmin, real_T ymin,
                          real_T dx, real_T dy, const real_T Z_map[1000000],
                          const struct0_T *geom, real_T dt, real_T dstate[9]);

emlrtCTX emlrtGetRootTLSGlobal(void);

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData);

/* End of code generation (FullStateDynamicsInC.h) */

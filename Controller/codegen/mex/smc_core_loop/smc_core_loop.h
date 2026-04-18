/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * smc_core_loop.h
 *
 * Code generation for function 'smc_core_loop'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "smc_core_loop_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
emlrtCTX emlrtGetRootTLSGlobal(void);

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData);

int32_T getThreadID(void);

void initThreadID(emlrtCTX aTLS);

void smc_core_loop(
    emlrtCTX aTLS, const real32_T K[3], real32_T eps_v, real32_T r, real32_T B,
    real32_T max_w, real32_T dt, real_T decim, const real32_T noise_scales[3],
    const real32_T mu_R_mega[15004], const real32_T mu_L_mega[15004],
    const real32_T t_X_all[720004], const real32_T t_Y_all[720004],
    const real32_T t_Th_all[720004], const real32_T t_Xd_all[720004],
    const real32_T t_Yd_all[720004], const real32_T t_Thd_all[720004], real_T M,
    real_T F, real_T N, real_T W, real_T n_steps,
    emxArray_real32_T *mega_max_spatial, emxArray_real32_T *mega_max_theta,
    emxArray_real32_T *sum_x, emxArray_real32_T *sum_y,
    emxArray_real32_T *sum_x2, emxArray_real32_T *sum_y2);

/* End of code generation (smc_core_loop.h) */

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * smc_core_loop.c
 *
 * Code generation for function 'smc_core_loop'
 *
 */

/* Include files */
#include "smc_core_loop.h"
#include "eml_rand.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "eml_randn.h"
#include "randn.h"
#include "rt_nonfinite.h"
#include "smc_core_loop_data.h"
#include "smc_core_loop_emxutil.h"
#include "smc_core_loop_types.h"
#include "tic.h"
#include "toc.h"
#include "emlrt.h"
#include "mwmathutil.h"
#include "omp.h"
#include <stdio.h>

/* Variable Definitions */
static int32_T threadID;
#pragma omp threadprivate(threadID)

/* Function Definitions */
emlrtCTX emlrtGetRootTLSGlobal(void)
{
  return emlrtRootTLSGlobal;
}

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData)
{
  omp_set_lock(&emlrtLockGlobal);
  emlrtCallLockeeFunction(aLockee, aTLS, aData);
  omp_unset_lock(&emlrtLockGlobal);
}

int32_T getThreadID(void)
{
  return threadID;
}

void initThreadID(emlrtCTX aTLS)
{
  jmp_buf *volatile emlrtJBStack;
  int32_T idx;
  int32_T initThreadID_numThreads;
  int32_T ub_loop;
  ub_loop = omp_get_max_threads();
  emlrtEnterParallelRegion(aTLS, omp_in_parallel());
  emlrtPushJmpBuf(aTLS, &emlrtJBStack);
  initThreadID_numThreads = emlrtAllocRegionTLSs(
      aTLS, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for schedule(static) num_threads(initThreadID_numThreads)

  for (idx = 1; idx <= ub_loop; idx++) {
    threadID = omp_get_thread_num();
  }
  emlrtPopJmpBuf(aTLS, &emlrtJBStack);
  emlrtExitParallelRegion(aTLS, omp_in_parallel());
}

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
    emxArray_real32_T *sum_x2, emxArray_real32_T *sum_y2)
{
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  emlrtTimespec expl_temp;
  emxArray_real32_T *loc_ms;
  emxArray_real32_T *loc_mt;
  emxArray_real32_T *loc_noise;
  emxArray_real32_T *loc_sx;
  emxArray_real32_T *loc_sx2;
  emxArray_real32_T *loc_sy;
  emxArray_real32_T *loc_sy2;
  emxArray_real32_T *loc_u;
  emxArray_real32_T *loc_x;
  emxArray_real32_T *mega_max_spatial_3D;
  emxArray_real32_T *mega_max_theta_3D;
  emxArray_real32_T *mu_L_2D;
  emxArray_real32_T *mu_R_2D;
  emxArray_real32_T *u_curr;
  emxArray_real32_T *x_curr;
  real_T P;
  real_T p;
  real_T p_base;
  real_T q;
  real_T step_timer_tv_nsec;
  real_T step_timer_tv_sec;
  int32_T b_loop_ub;
  int32_T b_m;
  int32_T b_unnamed_idx_2;
  int32_T c_i;
  int32_T c_loop_ub;
  int32_T c_th_tmp_tmp;
  int32_T d_loop_ub;
  int32_T e_loop_ub;
  int32_T f;
  int32_T i;
  int32_T loop_ub;
  int32_T m;
  int32_T n;
  int32_T ref_X_tmp;
  int32_T smc_core_loop_numThreads;
  int32_T unnamed_idx_0;
  int32_T unnamed_idx_2;
  real32_T K_1;
  real32_T K_2;
  real32_T K_3;
  real32_T ae3;
  real32_T c_th;
  real32_T c_th_tmp;
  real32_T e1;
  real32_T half_B;
  real32_T inv_2_epsv;
  real32_T inv_r;
  real32_T ns_1;
  real32_T ns_2;
  real32_T ns_3;
  real32_T r_over_2;
  real32_T r_over_B;
  real32_T ref_Th;
  real32_T ref_Thd;
  real32_T ref_X;
  real32_T ref_Xd;
  real32_T ref_Y;
  real32_T ref_Yd;
  real32_T s_3;
  real32_T s_th;
  real32_T th;
  real32_T v_nom_tmp;
  real32_T x1;
  real32_T x2;
  real32_T x3;
  real32_T *loc_ms_data;
  real32_T *loc_mt_data;
  real32_T *loc_noise_data;
  real32_T *loc_sx2_data;
  real32_T *loc_sx_data;
  real32_T *loc_sy2_data;
  real32_T *loc_sy_data;
  real32_T *loc_u_data;
  real32_T *loc_x_data;
  real32_T *mega_max_spatial_3D_data;
  real32_T *mega_max_theta_3D_data;
  real32_T *mu_L_2D_data;
  real32_T *mu_R_2D_data;
  real32_T *sum_x2_data;
  real32_T *sum_x_data;
  real32_T *sum_y2_data;
  real32_T *sum_y_data;
  real32_T *u_curr_data;
  real32_T *x_curr_data;
  uint32_T b_i;
  boolean_T emlrtHadParallelError = false;
  boolean_T update_ctrl;
  emlrtHeapReferenceStackEnterFcnR2012b(aTLS);
  /*  This function is lowkey black magic and took me like a solid 6 to 7 hours
   */
  /*  to code up with the help of Gemini. And it took a lot of redoing, because
   * the way */
  /*  the code is written is very very particular to the way memory is stored.
   */
  /*   */
  /*  "Load-bearing boilerplate" */
  /*  */
  /*  If you try doing all the MC stuff in a for loop, what you get is 4 nested
   */
  /*  for loops iterating over: */
  /*        - M trajectories */
  /*        - N iterations each */
  /*        - mu_r x mu_l testcases for each iteration */
  /*        - 3 states each (x,y,theta) */
  /*  */
  /*  and that sucks for speed. even if you parfor the outside, you can get */
  /*  everything down to maybe ~1.25 hours. The thing is, we can parfor each
   * step.  */
  /*  Batching makes it work, if you had a cache the size of your RAM. the issue
   * is cache misses,  */
  /*  and something called memory thrashing. When the array is so big it can't
   * physically fit */
  /*  inside your cache, the CPU spends a lot of time 1) searching for the */
  /*  data, 2) reorganizing the cache to do all of these math operations.  */
  /*  */
  /*  But more importantly, MATLAB does addition like this: A = B + C: */
  /*        1) create A the size of B (or C) which can be a million long */
  /*        2) go over each B and C idx, add them together save into A */
  /*  So the CPU tries making this faster by putting B and C into cache to do */
  /*  the math faster and reduce the "finding data" overhead. But if B and C */
  /*  are large, so is A, and they all try to fit in the cache at the same */
  /*  time. The CPU ends up taking data from RAM, filling the cache, and then */
  /*  sending the data back and taking the next chunk from RAM, which can be a
   */
  /*  lot of roundtrips for a single operation.  */
  /*  */
  /*  The question is how do we reduce the number of temporary allocations? */
  /*  Well, we reorder the way we do operations. If vectorization won't really
   */
  /*  work because the data size is too big, we do things sequentially.  */
  /*  */
  /*  If we do things sequentially, the order really doesn't matter, we can */
  /*  put the simulation on the inside, or on the outside. But considering that
   */
  /*  we have the fun fact that everything should be able to be parfored, with
   */
  /*  the exception of the physics simulation since each step is dependent on */
  /*  the previous state, we need to order things differently to take advantage
   */
  /*  of parfor.  */
  /*  */
  /*  Since everything inside a parfor has to be parallizable, we have to put */
  /*  the physics step on the outside, since it can't be parallizable; it has */
  /*  to be sequential since the previous state's data is necessary for the */
  /*  next state.  */
  /*  */
  /*  Next, we say that each trajectory gets a processor. At this point, it */
  /*  doesn't really matter which for loop goes on the outside. Putting the */
  /*  trajectory on the outside makes a lot of sense since it has the fewest */
  /*  number of iterations to complete for the most part, and since intuitively
   */
  /*  it makes sense to have each trajectory compute it's own testcases.  */
  /*  */
  /*  Now for each trajectory, we have to compute 1) the slip ratio/efficiency
   */
  /*  test cases, and 2) do that for every iteration. The order of this doesn't
   */
  /*  really matter. Either way we stride across data to get the next chunk of
   */
  /*  memory.  */
  /*   */
  /*  Since we want to operations on the current state of all the different */
  /*  testcases, we put that in column major so we can load that into cache as
   */
  /*  a contiguous block.  */
  /*  */
  /*  N iterations of F testcases gives me this many runs per trajectory */
  P = F * N;
  /*  Body Geometry */
  inv_r = 1.0F / r;
  r_over_2 = r / 2.0F;
  r_over_B = r / B;
  half_B = B / 2.0F;
  /*  Controller data */
  K_1 = K[0];
  K_2 = K[1];
  K_3 = K[2];
  inv_2_epsv = 1.0F / (2.0F * eps_v);
  /*  Noise data */
  ns_1 = noise_scales[0] * noise_scales[0];
  ns_2 = noise_scales[1] * noise_scales[1];
  ns_3 = noise_scales[2] * noise_scales[2];
  /*  State data */
  emxInit_real32_T(aTLS, &x_curr, 3);
  loop_ub = x_curr->size[0] * x_curr->size[1] * x_curr->size[2];
  x_curr->size[0] = 3;
  b_loop_ub = (int32_T)P;
  x_curr->size[1] = (int32_T)P;
  unnamed_idx_0 = (int32_T)M;
  x_curr->size[2] = (int32_T)M;
  emxEnsureCapacity_real32_T(x_curr, loop_ub);
  x_curr_data = x_curr->data;
  emxInit_real32_T(aTLS, &u_curr, 3);
  loop_ub = u_curr->size[0] * u_curr->size[1] * u_curr->size[2];
  u_curr->size[0] = 2;
  u_curr->size[1] = b_loop_ub;
  u_curr->size[2] = unnamed_idx_0;
  emxEnsureCapacity_real32_T(u_curr, loop_ub);
  u_curr_data = u_curr->data;
  loop_ub = ((int32_T)P << 1) * (int32_T)M;
  for (i = 0; i < loop_ub; i++) {
    u_curr_data[i] = 0.0F;
  }
  /*  This doesn't matter as long as it aligns with the state data */
  /*  representation so we can query with the same idx */
  emxInit_real32_T(aTLS, &mu_R_2D, 2);
  loop_ub = mu_R_2D->size[0] * mu_R_2D->size[1];
  mu_R_2D->size[0] = b_loop_ub;
  mu_R_2D->size[1] = unnamed_idx_0;
  emxEnsureCapacity_real32_T(mu_R_2D, loop_ub);
  mu_R_2D_data = mu_R_2D->data;
  c_loop_ub = (int32_T)P * (int32_T)M;
  for (i = 0; i < c_loop_ub; i++) {
    mu_R_2D_data[i] = mu_R_mega[i];
  }
  emxInit_real32_T(aTLS, &mu_L_2D, 2);
  loop_ub = mu_L_2D->size[0] * mu_L_2D->size[1];
  mu_L_2D->size[0] = b_loop_ub;
  mu_L_2D->size[1] = unnamed_idx_0;
  emxEnsureCapacity_real32_T(mu_L_2D, loop_ub);
  mu_L_2D_data = mu_L_2D->data;
  for (i = 0; i < c_loop_ub; i++) {
    mu_L_2D_data[i] = mu_L_mega[i];
  }
  /*  this is where we load the data into column major form, storing all */
  /*  the states for a trajectory and all its testcases into one contiguous */
  /*  array in memory */
  for (m = 0; m < unnamed_idx_0; m++) {
    for (th = t_Th_all[m]; th >= 6.28318548F; th -= 6.28318548F) {
    }
    while (th < 0.0F) {
      th += 6.28318548F;
    }
    for (i = 0; i < b_loop_ub; i++) {
      x_curr_data[3 * i + 3 * x_curr->size[1] * m] = t_X_all[m];
    }
    for (i = 0; i < b_loop_ub; i++) {
      x_curr_data[(3 * i + 3 * x_curr->size[1] * m) + 1] = t_Y_all[m];
    }
    for (i = 0; i < b_loop_ub; i++) {
      x_curr_data[(3 * i + 3 * x_curr->size[1] * m) + 2] = th;
    }
  }
  /*  Accumulators */
  emxInit_real32_T(aTLS, &mega_max_spatial_3D, 2);
  loop_ub = mega_max_spatial_3D->size[0] * mega_max_spatial_3D->size[1];
  mega_max_spatial_3D->size[0] = b_loop_ub;
  mega_max_spatial_3D->size[1] = unnamed_idx_0;
  emxEnsureCapacity_real32_T(mega_max_spatial_3D, loop_ub);
  mega_max_spatial_3D_data = mega_max_spatial_3D->data;
  for (i = 0; i < c_loop_ub; i++) {
    mega_max_spatial_3D_data[i] = 0.0F;
  }
  emxInit_real32_T(aTLS, &mega_max_theta_3D, 2);
  loop_ub = mega_max_theta_3D->size[0] * mega_max_theta_3D->size[1];
  mega_max_theta_3D->size[0] = b_loop_ub;
  mega_max_theta_3D->size[1] = unnamed_idx_0;
  emxEnsureCapacity_real32_T(mega_max_theta_3D, loop_ub);
  mega_max_theta_3D_data = mega_max_theta_3D->data;
  for (i = 0; i < c_loop_ub; i++) {
    mega_max_theta_3D_data[i] = 0.0F;
  }
  loop_ub = (int32_T)F;
  unnamed_idx_2 = (int32_T)n_steps;
  b_loop_ub = (int32_T)F;
  b_unnamed_idx_2 = (int32_T)n_steps;
  /*  Time loop */
  expl_temp = tic();
  step_timer_tv_sec = expl_temp.tv_sec;
  step_timer_tv_nsec = expl_temp.tv_nsec;
  c_loop_ub = sum_x->size[0] * sum_x->size[1] * sum_x->size[2];
  sum_x->size[0] = unnamed_idx_0;
  sum_x->size[1] = loop_ub;
  sum_x->size[2] = unnamed_idx_2;
  emxEnsureCapacity_real32_T(sum_x, c_loop_ub);
  sum_x_data = sum_x->data;
  loop_ub = sum_y->size[0] * sum_y->size[1] * sum_y->size[2];
  sum_y->size[0] = unnamed_idx_0;
  sum_y->size[1] = b_loop_ub;
  sum_y->size[2] = b_unnamed_idx_2;
  emxEnsureCapacity_real32_T(sum_y, loop_ub);
  sum_y_data = sum_y->data;
  loop_ub = sum_x2->size[0] * sum_x2->size[1] * sum_x2->size[2];
  sum_x2->size[0] = unnamed_idx_0;
  sum_x2->size[1] = b_loop_ub;
  sum_x2->size[2] = b_unnamed_idx_2;
  emxEnsureCapacity_real32_T(sum_x2, loop_ub);
  sum_x2_data = sum_x2->data;
  loop_ub = sum_y2->size[0] * sum_y2->size[1] * sum_y2->size[2];
  sum_y2->size[0] = unnamed_idx_0;
  sum_y2->size[1] = b_loop_ub;
  sum_y2->size[2] = b_unnamed_idx_2;
  emxEnsureCapacity_real32_T(sum_y2, loop_ub);
  sum_y2_data = sum_y2->data;
  for (i = 0; i < unnamed_idx_2; i++) {
    b_i = (uint32_T)i + 1U;
    if (decim == 0.0) {
      q = ((real_T)i + 1.0) - 1.0;
      if (i == 0) {
        q = decim;
      }
    } else if (muDoubleScalarIsNaN(decim)) {
      q = rtNaN;
    } else if (muDoubleScalarIsInf(decim)) {
      if (decim > 0.0) {
        if (i > 0) {
          q = ((real_T)i + 1.0) - 1.0;
        } else {
          q = 0.0;
        }
      } else if (i > 0) {
        q = decim;
      } else {
        q = -0.0;
      }
    } else {
      if (decim > muDoubleScalarFloor(decim)) {
        q = muDoubleScalarAbs((((real_T)i + 1.0) - 1.0) / decim);
        if (muDoubleScalarAbs(q - muDoubleScalarFloor(q + 0.5)) >
            2.2204460492503131E-16 * q) {
          q = muDoubleScalarRem(((real_T)i + 1.0) - 1.0, decim);
        } else {
          q = 0.0;
        }
      } else {
        q = muDoubleScalarRem(((real_T)i + 1.0) - 1.0, decim);
      }
      if (q == 0.0) {
        q = decim * 0.0;
      } else if ((q > 0.0) && (decim < 0.0)) {
        q += decim;
      }
    }
    update_ctrl = (q == 0.0);
    /*  Trajectory loop, do this parallel */
    emlrtEnterParallelRegion(aTLS, omp_in_parallel());
    emlrtPushJmpBuf(aTLS, &emlrtJBStack);
    if (!omp_in_parallel()) {
      c_eml_rand_mt19937ar_stateful_s(true);
      eml_rand_swap();
      eml_randn_swap(true);
    }
    smc_core_loop_numThreads = emlrtAllocRegionTLSs(
        aTLS, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(smc_core_loop_numThreads) private(            \
        loc_x_data, loc_u_data, loc_ms_data, loc_mt_data, loc_sx_data,         \
            loc_sy_data, loc_sx2_data, loc_sy2_data, loc_noise_data, loc_x,    \
            loc_u, loc_ms, loc_mt, loc_sx, loc_sy, loc_sx2, loc_sy2,           \
            loc_noise, ae3, e1, x3, x2, x1, s_3, s_th, c_th, p, p_base,        \
            ref_Thd, ref_Yd, ref_Xd, ref_Th, ref_Y, ref_X, ref_X_tmp,          \
            d_loop_ub, f, e_loop_ub, c_i, n, c_th_tmp_tmp, c_th_tmp,           \
            v_nom_tmp, emlrtJBEnviron)                                         \
    firstprivate(aTLS, emlrtHadParallelError)
    {
      aTLS = emlrtAllocTLS(aTLS, omp_get_thread_num());
      emlrtSetJmpBuf(aTLS, &emlrtJBEnviron);
      if (setjmp(emlrtJBEnviron) == 0) {
        emxInit_real32_T(aTLS, &loc_x, 2);
        emxInit_real32_T(aTLS, &loc_u, 2);
        emxInit_real32_T(aTLS, &loc_ms, 1);
        emxInit_real32_T(aTLS, &loc_mt, 1);
        emxInit_real32_T(aTLS, &loc_sx, 2);
        emxInit_real32_T(aTLS, &loc_sy, 2);
        emxInit_real32_T(aTLS, &loc_sx2, 2);
        emxInit_real32_T(aTLS, &loc_sy2, 2);
        emxInit_real32_T(aTLS, &loc_noise, 2);
      } else {
        emlrtHadParallelError = true;
      }
#pragma omp for nowait
      for (b_m = 0; b_m < unnamed_idx_0; b_m++) {
        if (emlrtHadParallelError) {
          continue;
        }
        if (setjmp(emlrtJBEnviron) == 0) {
          /*  desired trajectory */
          ref_X_tmp = b_m + (((int32_T)b_i - 1) << 2);
          ref_X = t_X_all[ref_X_tmp];
          ref_Y = t_Y_all[ref_X_tmp];
          /*  C-safe wrap 2-pi that's very fast */
          for (ref_Th = t_Th_all[ref_X_tmp]; ref_Th >= 6.28318548F;
               ref_Th -= 6.28318548F) {
          }
          while (ref_Th < 0.0F) {
            ref_Th += 6.28318548F;
          }
          /*  Get the desired derivatives */
          ref_Xd = 0.0F;
          ref_Yd = 0.0F;
          ref_Thd = 0.0F;
          if (update_ctrl) {
            ref_Xd = t_Xd_all[ref_X_tmp];
            ref_Yd = t_Yd_all[ref_X_tmp];
            ref_Thd = t_Thd_all[ref_X_tmp];
          }
          /*  copy the slices to do math on them, and reconcile after the */
          /*  process is done (m is the trajectory!) */
          ref_X_tmp = loc_x->size[0] * loc_x->size[1];
          loc_x->size[0] = 3;
          d_loop_ub = x_curr->size[1];
          loc_x->size[1] = x_curr->size[1];
          emxEnsureCapacity_real32_T(loc_x, ref_X_tmp);
          loc_x_data = loc_x->data;
          for (f = 0; f < d_loop_ub; f++) {
            loc_x_data[3 * f] = x_curr_data[3 * f + 3 * x_curr->size[1] * b_m];
            loc_x_data[3 * f + 1] =
                x_curr_data[(3 * f + 3 * x_curr->size[1] * b_m) + 1];
            loc_x_data[3 * f + 2] =
                x_curr_data[(3 * f + 3 * x_curr->size[1] * b_m) + 2];
          }
          ref_X_tmp = loc_u->size[0] * loc_u->size[1];
          loc_u->size[0] = 2;
          loc_u->size[1] = x_curr->size[1];
          emxEnsureCapacity_real32_T(loc_u, ref_X_tmp);
          loc_u_data = loc_u->data;
          for (f = 0; f < d_loop_ub; f++) {
            loc_u_data[2 * f] = u_curr_data[2 * f + 2 * u_curr->size[1] * b_m];
            loc_u_data[2 * f + 1] =
                u_curr_data[(2 * f + 2 * u_curr->size[1] * b_m) + 1];
          }
          ref_X_tmp = loc_ms->size[0];
          loc_ms->size[0] = x_curr->size[1];
          emxEnsureCapacity_real32_T(loc_ms, ref_X_tmp);
          loc_ms_data = loc_ms->data;
          for (f = 0; f < d_loop_ub; f++) {
            loc_ms_data[f] =
                mega_max_spatial_3D_data[f +
                                         mega_max_spatial_3D->size[0] * b_m];
          }
          ref_X_tmp = loc_mt->size[0];
          loc_mt->size[0] = x_curr->size[1];
          emxEnsureCapacity_real32_T(loc_mt, ref_X_tmp);
          loc_mt_data = loc_mt->data;
          for (f = 0; f < d_loop_ub; f++) {
            loc_mt_data[f] =
                mega_max_theta_3D_data[f + mega_max_theta_3D->size[0] * b_m];
          }
          ref_X_tmp = loc_sx->size[0] * loc_sx->size[1];
          loc_sx->size[0] = 1;
          e_loop_ub = (int32_T)F;
          loc_sx->size[1] = (int32_T)F;
          emxEnsureCapacity_real32_T(loc_sx, ref_X_tmp);
          loc_sx_data = loc_sx->data;
          ref_X_tmp = loc_sy->size[0] * loc_sy->size[1];
          loc_sy->size[0] = 1;
          loc_sy->size[1] = (int32_T)F;
          emxEnsureCapacity_real32_T(loc_sy, ref_X_tmp);
          loc_sy_data = loc_sy->data;
          ref_X_tmp = loc_sx2->size[0] * loc_sx2->size[1];
          loc_sx2->size[0] = 1;
          loc_sx2->size[1] = (int32_T)F;
          emxEnsureCapacity_real32_T(loc_sx2, ref_X_tmp);
          loc_sx2_data = loc_sx2->data;
          ref_X_tmp = loc_sy2->size[0] * loc_sy2->size[1];
          loc_sy2->size[0] = 1;
          loc_sy2->size[1] = (int32_T)F;
          emxEnsureCapacity_real32_T(loc_sy2, ref_X_tmp);
          loc_sy2_data = loc_sy2->data;
          for (f = 0; f < e_loop_ub; f++) {
            loc_sx_data[f] = 0.0F;
            loc_sy_data[f] = 0.0F;
            loc_sx2_data[f] = 0.0F;
            loc_sy2_data[f] = 0.0F;
          }
          randn(P, loc_noise);
          loc_noise_data = loc_noise->data;
          /*  slip ratio testcase */
          for (f = 0; f < e_loop_ub; f++) {
            p_base = (((real_T)f + 1.0) - 1.0) * N;
            /*  run iterations */
            c_i = (int32_T)N;
            for (n = 0; n < c_i; n++) {
              p = p_base + ((real_T)n + 1.0);
              c_th_tmp_tmp = 3 * ((int32_T)p - 1);
              c_th_tmp = loc_x_data[c_th_tmp_tmp + 2];
              c_th = muSingleScalarCos(c_th_tmp);
              s_th = muSingleScalarSin(c_th_tmp);
              /*  Control update. all this stuff is just the same stuff */
              /*  i've been saying, but unrolled so that C is happy */
              /*  doing sequential operations. No vectorization here. */
              /*  just math. */
              if (update_ctrl) {
                e1 = loc_x_data[c_th_tmp_tmp] - ref_X;
                x1 = loc_x_data[c_th_tmp_tmp + 1] - ref_Y;
                for (s_3 = c_th_tmp - ref_Th; s_3 > 3.14159274F;
                     s_3 -= 6.28318548F) {
                }
                while (s_3 < -3.14159274F) {
                  s_3 += 6.28318548F;
                }
                ref_X_tmp = 2 * ((int32_T)p - 1);
                v_nom_tmp = loc_u_data[ref_X_tmp];
                ae3 = loc_u_data[ref_X_tmp + 1];
                x2 = r_over_2 * (v_nom_tmp + ae3);
                ae3 = r_over_B * (v_nom_tmp - ae3);
                x3 = (-K_1 * muSingleScalarTanh(e1 * inv_2_epsv) - x2 * c_th) +
                     ref_Xd;
                e1 = (-K_2 * muSingleScalarTanh(x1 * inv_2_epsv) - x2 * s_th) +
                     ref_Yd;
                ae3 = (-K_3 * muSingleScalarTanh(s_3 * inv_2_epsv) - ae3) +
                      ref_Thd;
                e1 = x3 * c_th + e1 * s_th;
                /*  this is the clipping function, just made... into */
                /*  this... */
                ae3 *= half_B;
                loc_u_data[ref_X_tmp] = muSingleScalarMax(
                    muSingleScalarMin(v_nom_tmp + (e1 + ae3) * inv_r, max_w),
                    -max_w);
                loc_u_data[ref_X_tmp + 1] = muSingleScalarMax(
                    muSingleScalarMin(
                        loc_u_data[ref_X_tmp + 1] + (e1 - ae3) * inv_r, max_w),
                    -max_w);
              }
              /*  Apply the control */
              ref_X_tmp = 2 * ((int32_T)p - 1);
              x3 = mu_R_2D_data[((int32_T)p + mu_R_2D->size[0] * b_m) - 1] *
                   loc_u_data[ref_X_tmp];
              e1 = mu_L_2D_data[((int32_T)p + mu_L_2D->size[0] * b_m) - 1] *
                   loc_u_data[ref_X_tmp + 1];
              ae3 = r_over_2 * (x3 + e1);
              x1 = loc_x_data[c_th_tmp_tmp] +
                   (ae3 * c_th + ns_1 * loc_noise_data[c_th_tmp_tmp]) * dt;
              x2 = loc_x_data[c_th_tmp_tmp + 1] +
                   (ae3 * s_th + ns_2 * loc_noise_data[c_th_tmp_tmp + 1]) * dt;
              for (x3 = c_th_tmp + (r_over_B * (x3 - e1) +
                                    ns_3 * loc_noise_data[c_th_tmp_tmp + 2]) *
                                       dt;
                   x3 >= 6.28318548F; x3 -= 6.28318548F) {
              }
              while (x3 < 0.0F) {
                x3 += 6.28318548F;
              }
              loc_x_data[c_th_tmp_tmp] = x1;
              loc_x_data[c_th_tmp_tmp + 1] = x2;
              loc_x_data[c_th_tmp_tmp + 2] = x3;
              /*  statistics calcs */
              /*  envelope calculation */
              e1 = x1 - ref_X;
              ae3 = x2 - ref_Y;
              ae3 = e1 * e1 + ae3 * ae3;
              e1 = loc_ms_data[(int32_T)p - 1];
              if (ae3 > e1 * e1) {
                loc_ms_data[(int32_T)p - 1] = muSingleScalarSqrt(ae3);
              }
              for (ae3 = x3 - ref_Th; ae3 > 3.14159274F; ae3 -= 6.28318548F) {
              }
              while (ae3 < -3.14159274F) {
                ae3 += 6.28318548F;
              }
              ae3 = muSingleScalarAbs(ae3);
              if (ae3 > loc_mt_data[(int32_T)p - 1]) {
                loc_mt_data[(int32_T)p - 1] = ae3;
              }
              /*  stoer the stats */
              loc_sx_data[f] += x1;
              loc_sy_data[f] += x2;
              loc_sx2_data[f] += x1 * x1;
              loc_sy2_data[f] += x2 * x2;
            }
          }
          /*  copy the data back */
          for (f = 0; f < d_loop_ub; f++) {
            x_curr_data[3 * f + 3 * x_curr->size[1] * b_m] = loc_x_data[3 * f];
            x_curr_data[(3 * f + 3 * x_curr->size[1] * b_m) + 1] =
                loc_x_data[3 * f + 1];
            x_curr_data[(3 * f + 3 * x_curr->size[1] * b_m) + 2] =
                loc_x_data[3 * f + 2];
          }
          for (f = 0; f < d_loop_ub; f++) {
            u_curr_data[2 * f + 2 * u_curr->size[1] * b_m] = loc_u_data[2 * f];
            u_curr_data[(2 * f + 2 * u_curr->size[1] * b_m) + 1] =
                loc_u_data[2 * f + 1];
          }
          for (f = 0; f < d_loop_ub; f++) {
            mega_max_spatial_3D_data[f + mega_max_spatial_3D->size[0] * b_m] =
                loc_ms_data[f];
          }
          for (f = 0; f < d_loop_ub; f++) {
            mega_max_theta_3D_data[f + mega_max_theta_3D->size[0] * b_m] =
                loc_mt_data[f];
          }
          for (f = 0; f < e_loop_ub; f++) {
            sum_x_data[(b_m + sum_x->size[0] * f) +
                       sum_x->size[0] * sum_x->size[1] * ((int32_T)b_i - 1)] =
                loc_sx_data[f];
          }
          for (f = 0; f < e_loop_ub; f++) {
            sum_y_data[(b_m + sum_y->size[0] * f) +
                       sum_y->size[0] * sum_y->size[1] * ((int32_T)b_i - 1)] =
                loc_sy_data[f];
            sum_x2_data[(b_m + sum_x2->size[0] * f) +
                        sum_x2->size[0] * sum_x2->size[1] *
                            ((int32_T)b_i - 1)] = loc_sx2_data[f];
            sum_y2_data[(b_m + sum_y2->size[0] * f) +
                        sum_y2->size[0] * sum_y2->size[1] *
                            ((int32_T)b_i - 1)] = loc_sy2_data[f];
          }
        } else {
          emlrtHadParallelError = true;
        }
      }
      if (!emlrtHadParallelError) {
        emlrtHeapReferenceStackLeaveScope(aTLS, 9);
        emxFree_real32_T(aTLS, &loc_noise);
        emxFree_real32_T(aTLS, &loc_sy2);
        emxFree_real32_T(aTLS, &loc_sx2);
        emxFree_real32_T(aTLS, &loc_sy);
        emxFree_real32_T(aTLS, &loc_sx);
        emxFree_real32_T(aTLS, &loc_mt);
        emxFree_real32_T(aTLS, &loc_ms);
        emxFree_real32_T(aTLS, &loc_u);
        emxFree_real32_T(aTLS, &loc_x);
      }
    }
    if (!omp_in_parallel()) {
      c_eml_rand_mt19937ar_stateful_s(false);
      eml_rand_swap();
      eml_randn_swap(false);
    }
    emlrtPopJmpBuf(aTLS, &emlrtJBStack);
    emlrtExitParallelRegion(aTLS, omp_in_parallel());
    if (muDoubleScalarRem(b_i, 1000.0) == 0.0) {
      q = toc(step_timer_tv_sec, step_timer_tv_nsec);
      mexPrintf("step %.1f / %.1f  (%.2f s per 1k)\n", (real_T)b_i, n_steps, q);
      expl_temp = tic();
      step_timer_tv_sec = expl_temp.tv_sec;
      step_timer_tv_nsec = expl_temp.tv_nsec;
    }
  }
  emxFree_real32_T(aTLS, &mu_L_2D);
  emxFree_real32_T(aTLS, &mu_R_2D);
  emxFree_real32_T(aTLS, &u_curr);
  emxFree_real32_T(aTLS, &x_curr);
  loop_ub = mega_max_spatial->size[0] * mega_max_spatial->size[1];
  mega_max_spatial->size[0] = 1;
  c_loop_ub = (int32_T)W;
  mega_max_spatial->size[1] = (int32_T)W;
  emxEnsureCapacity_real32_T(mega_max_spatial, loop_ub);
  x_curr_data = mega_max_spatial->data;
  for (i = 0; i < c_loop_ub; i++) {
    x_curr_data[i] = mega_max_spatial_3D_data[i];
  }
  emxFree_real32_T(aTLS, &mega_max_spatial_3D);
  loop_ub = mega_max_theta->size[0] * mega_max_theta->size[1];
  mega_max_theta->size[0] = 1;
  mega_max_theta->size[1] = c_loop_ub;
  emxEnsureCapacity_real32_T(mega_max_theta, loop_ub);
  x_curr_data = mega_max_theta->data;
  for (i = 0; i < c_loop_ub; i++) {
    x_curr_data[i] = mega_max_theta_3D_data[i];
  }
  emxFree_real32_T(aTLS, &mega_max_theta_3D);
  emlrtHeapReferenceStackLeaveFcnR2012b(aTLS);
}

/* End of code generation (smc_core_loop.c) */

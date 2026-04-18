/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_smc_core_loop_api.c
 *
 * Code generation for function '_coder_smc_core_loop_api'
 *
 */

/* Include files */
#include "_coder_smc_core_loop_api.h"
#include "rt_nonfinite.h"
#include "smc_core_loop.h"
#include "smc_core_loop_data.h"
#include "smc_core_loop_emxutil.h"
#include "smc_core_loop_types.h"

/* Function Declarations */
static real32_T (*b_emlrt_marshallIn(const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[3];

static const mxArray *b_emlrt_marshallOut(emxArray_real32_T *u);

static real32_T c_emlrt_marshallIn(const mxArray *nullptr,
                                   const char_T *identifier);

static real32_T d_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static real_T e_emlrt_marshallIn(const mxArray *nullptr,
                                 const char_T *identifier);

static real32_T (*emlrt_marshallIn(const mxArray *nullptr,
                                   const char_T *identifier))[3];

static const mxArray *emlrt_marshallOut(emxArray_real32_T *u);

static real_T f_emlrt_marshallIn(const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real32_T (*g_emlrt_marshallIn(const mxArray *nullptr,
                                     const char_T *identifier))[15004];

static real32_T (*h_emlrt_marshallIn(
    const mxArray *u, const emlrtMsgIdentifier *parentId))[15004];

static real32_T (*i_emlrt_marshallIn(const mxArray *nullptr,
                                     const char_T *identifier))[720004];

static real32_T (*j_emlrt_marshallIn(
    const mxArray *u, const emlrtMsgIdentifier *parentId))[720004];

static real32_T (*k_emlrt_marshallIn(const mxArray *src,
                                     const emlrtMsgIdentifier *msgId))[3];

static real32_T l_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId);

static real_T m_emlrt_marshallIn(const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real32_T (*n_emlrt_marshallIn(const mxArray *src,
                                     const emlrtMsgIdentifier *msgId))[15004];

static real32_T (*o_emlrt_marshallIn(const mxArray *src,
                                     const emlrtMsgIdentifier *msgId))[720004];

/* Function Definitions */
static real32_T (*b_emlrt_marshallIn(const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[3]
{
  real32_T(*y)[3];
  y = k_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_emlrt_marshallOut(emxArray_real32_T *u)
{
  static const int32_T iv[3] = {0, 0, 0};
  const mxArray *m;
  const mxArray *y;
  real32_T *u_data;
  u_data = u->data;
  y = NULL;
  m = emlrtCreateNumericArray(3, (const void *)&iv[0], mxSINGLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u_data[0]);
  emlrtSetDimensions((mxArray *)m, &u->size[0], 3);
  u->canFreeData = false;
  emlrtAssign(&y, m);
  return y;
}

static real32_T c_emlrt_marshallIn(const mxArray *nullptr,
                                   const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real32_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static real32_T d_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  real32_T y;
  y = l_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T e_emlrt_marshallIn(const mxArray *nullptr,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static real32_T (*emlrt_marshallIn(const mxArray *nullptr,
                                   const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real32_T(*y)[3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static const mxArray *emlrt_marshallOut(emxArray_real32_T *u)
{
  static const int32_T iv[2] = {0, 0};
  const mxArray *m;
  const mxArray *y;
  real32_T *u_data;
  u_data = u->data;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxSINGLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u_data[0]);
  emlrtSetDimensions((mxArray *)m, &u->size[0], 2);
  u->canFreeData = false;
  emlrtAssign(&y, m);
  return y;
}

static real_T f_emlrt_marshallIn(const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = m_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real32_T (*g_emlrt_marshallIn(const mxArray *nullptr,
                                     const char_T *identifier))[15004]
{
  emlrtMsgIdentifier thisId;
  real32_T(*y)[15004];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static real32_T (*h_emlrt_marshallIn(const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[15004]
{
  real32_T(*y)[15004];
  y = n_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real32_T (*i_emlrt_marshallIn(const mxArray *nullptr,
                                     const char_T *identifier))[720004]
{
  emlrtMsgIdentifier thisId;
  real32_T(*y)[720004];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = j_emlrt_marshallIn(emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static real32_T (*j_emlrt_marshallIn(
    const mxArray *u, const emlrtMsgIdentifier *parentId))[720004]
{
  real32_T(*y)[720004];
  y = o_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real32_T (*k_emlrt_marshallIn(const mxArray *src,
                                     const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims = 3;
  int32_T i;
  real32_T(*ret)[3];
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "single", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real32_T(*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real32_T l_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real32_T ret;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "single", false, 0U,
                          (const void *)&dims);
  ret = *(real32_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T m_emlrt_marshallIn(const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real32_T (*n_emlrt_marshallIn(const mxArray *src,
                                     const emlrtMsgIdentifier *msgId))[15004]
{
  static const int32_T dims[2] = {1, 15004};
  int32_T iv[2];
  real32_T(*ret)[15004];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "single", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real32_T(*)[15004])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real32_T (*o_emlrt_marshallIn(const mxArray *src,
                                     const emlrtMsgIdentifier *msgId))[720004]
{
  static const int32_T dims[2] = {4, 180001};
  int32_T iv[2];
  real32_T(*ret)[720004];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "single", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real32_T(*)[720004])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void smc_core_loop_api(const mxArray *const prhs[21], int32_T nlhs,
                       const mxArray *plhs[6])
{
  emxArray_real32_T *mega_max_spatial;
  emxArray_real32_T *mega_max_theta;
  emxArray_real32_T *sum_x;
  emxArray_real32_T *sum_x2;
  emxArray_real32_T *sum_y;
  emxArray_real32_T *sum_y2;
  real_T F;
  real_T M;
  real_T N;
  real_T W;
  real_T decim;
  real_T n_steps;
  real32_T(*t_Th_all)[720004];
  real32_T(*t_Thd_all)[720004];
  real32_T(*t_X_all)[720004];
  real32_T(*t_Xd_all)[720004];
  real32_T(*t_Y_all)[720004];
  real32_T(*t_Yd_all)[720004];
  real32_T(*mu_L_mega)[15004];
  real32_T(*mu_R_mega)[15004];
  real32_T(*K)[3];
  real32_T(*noise_scales)[3];
  real32_T B;
  real32_T dt;
  real32_T eps_v;
  real32_T max_w;
  real32_T r;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  /* Marshall function inputs */
  K = emlrt_marshallIn(emlrtAlias(prhs[0]), "K");
  eps_v = c_emlrt_marshallIn(emlrtAliasP(prhs[1]), "eps_v");
  r = c_emlrt_marshallIn(emlrtAliasP(prhs[2]), "r");
  B = c_emlrt_marshallIn(emlrtAliasP(prhs[3]), "B");
  max_w = c_emlrt_marshallIn(emlrtAliasP(prhs[4]), "max_w");
  dt = c_emlrt_marshallIn(emlrtAliasP(prhs[5]), "dt");
  decim = e_emlrt_marshallIn(emlrtAliasP(prhs[6]), "decim");
  noise_scales = emlrt_marshallIn(emlrtAlias(prhs[7]), "noise_scales");
  mu_R_mega = g_emlrt_marshallIn(emlrtAlias(prhs[8]), "mu_R_mega");
  mu_L_mega = g_emlrt_marshallIn(emlrtAlias(prhs[9]), "mu_L_mega");
  t_X_all = i_emlrt_marshallIn(emlrtAlias(prhs[10]), "t_X_all");
  t_Y_all = i_emlrt_marshallIn(emlrtAlias(prhs[11]), "t_Y_all");
  t_Th_all = i_emlrt_marshallIn(emlrtAlias(prhs[12]), "t_Th_all");
  t_Xd_all = i_emlrt_marshallIn(emlrtAlias(prhs[13]), "t_Xd_all");
  t_Yd_all = i_emlrt_marshallIn(emlrtAlias(prhs[14]), "t_Yd_all");
  t_Thd_all = i_emlrt_marshallIn(emlrtAlias(prhs[15]), "t_Thd_all");
  M = e_emlrt_marshallIn(emlrtAliasP(prhs[16]), "M");
  F = e_emlrt_marshallIn(emlrtAliasP(prhs[17]), "F");
  N = e_emlrt_marshallIn(emlrtAliasP(prhs[18]), "N");
  W = e_emlrt_marshallIn(emlrtAliasP(prhs[19]), "W");
  n_steps = e_emlrt_marshallIn(emlrtAliasP(prhs[20]), "n_steps");
  /* Invoke the target function */
  emxInit_real32_T(emlrtRootTLSGlobal, &mega_max_spatial, 2);
  emxInit_real32_T(emlrtRootTLSGlobal, &mega_max_theta, 2);
  emxInit_real32_T(emlrtRootTLSGlobal, &sum_x, 3);
  emxInit_real32_T(emlrtRootTLSGlobal, &sum_y, 3);
  emxInit_real32_T(emlrtRootTLSGlobal, &sum_x2, 3);
  emxInit_real32_T(emlrtRootTLSGlobal, &sum_y2, 3);
  smc_core_loop(emlrtRootTLSGlobal, *K, eps_v, r, B, max_w, dt, decim,
                *noise_scales, *mu_R_mega, *mu_L_mega, *t_X_all, *t_Y_all,
                *t_Th_all, *t_Xd_all, *t_Yd_all, *t_Thd_all, M, F, N, W,
                n_steps, mega_max_spatial, mega_max_theta, sum_x, sum_y, sum_x2,
                sum_y2);
  /* Marshall function outputs */
  mega_max_spatial->canFreeData = false;
  plhs[0] = emlrt_marshallOut(mega_max_spatial);
  emxFree_real32_T(emlrtRootTLSGlobal, &mega_max_spatial);
  if (nlhs > 1) {
    mega_max_theta->canFreeData = false;
    plhs[1] = emlrt_marshallOut(mega_max_theta);
  }
  emxFree_real32_T(emlrtRootTLSGlobal, &mega_max_theta);
  if (nlhs > 2) {
    sum_x->canFreeData = false;
    plhs[2] = b_emlrt_marshallOut(sum_x);
  }
  emxFree_real32_T(emlrtRootTLSGlobal, &sum_x);
  if (nlhs > 3) {
    sum_y->canFreeData = false;
    plhs[3] = b_emlrt_marshallOut(sum_y);
  }
  emxFree_real32_T(emlrtRootTLSGlobal, &sum_y);
  if (nlhs > 4) {
    sum_x2->canFreeData = false;
    plhs[4] = b_emlrt_marshallOut(sum_x2);
  }
  emxFree_real32_T(emlrtRootTLSGlobal, &sum_x2);
  if (nlhs > 5) {
    sum_y2->canFreeData = false;
    plhs[5] = b_emlrt_marshallOut(sum_y2);
  }
  emxFree_real32_T(emlrtRootTLSGlobal, &sum_y2);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (_coder_smc_core_loop_api.c) */

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * FullStateDynamicsInC.c
 *
 * Code generation for function 'FullStateDynamicsInC'
 *
 */

/* Include files */
#include "FullStateDynamicsInC.h"
#include "FullStateDynamicsInC_data.h"
#include "FullStateDynamicsInC_types.h"
#include "rt_nonfinite.h"
#include "warning.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>
#include <string.h>

/* Type Definitions */
#ifndef typedef_captured_var
#define typedef_captured_var
typedef struct {
  real_T contents;
} captured_var;
#endif /* typedef_captured_var */

#ifndef typedef_c_captured_var
#define typedef_c_captured_var
typedef struct {
  real_T contents[8];
} c_captured_var;
#endif /* typedef_c_captured_var */

/* Variable Definitions */
static emlrtRSInfo b_emlrtRSI =
    {
        30,                     /* lineNo */
        "FullStateDynamicsInC", /* fcnName */
        "C:"
        "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsI"
        "nC.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI =
    {
        31,                     /* lineNo */
        "FullStateDynamicsInC", /* fcnName */
        "C:"
        "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsI"
        "nC.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI =
    {
        33,                     /* lineNo */
        "FullStateDynamicsInC", /* fcnName */
        "C:"
        "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsI"
        "nC.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI =
    {
        34,                     /* lineNo */
        "FullStateDynamicsInC", /* fcnName */
        "C:"
        "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsI"
        "nC.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI =
    {
        36,                     /* lineNo */
        "FullStateDynamicsInC", /* fcnName */
        "C:"
        "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsI"
        "nC.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI =
    {
        37,                     /* lineNo */
        "FullStateDynamicsInC", /* fcnName */
        "C:"
        "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsI"
        "nC.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI =
    {
        61,                         /* lineNo */
        "FullStateDynamicsInC/att", /* fcnName */
        "C:"
        "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsI"
        "nC.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI =
    {
        67,                         /* lineNo */
        "FullStateDynamicsInC/att", /* fcnName */
        "C:"
        "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsI"
        "nC.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    20,         /* lineNo */
    "mldivide", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pathName
                                                                         */
};

static emlrtRSInfo l_emlrtRSI = {
    42,      /* lineNo */
    "mldiv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pathName
                                                                         */
};

static emlrtRSInfo
    m_emlrtRSI =
        {
            61,        /* lineNo */
            "lusolve", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtRSInfo
    n_emlrtRSI =
        {
            293,          /* lineNo */
            "lusolve3x3", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtRSInfo
    o_emlrtRSI =
        {
            90,              /* lineNo */
            "warn_singular", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtBCInfo emlrtBCI = {
    1,                                           /* iFirst */
    1000,                                        /* iLast */
    89,                                          /* lineNo */
    22,                                          /* colNo */
    "Z_map",                                     /* aName */
    "FullStateDynamicsInC/fast_triangle_interp", /* fName */
    "C:"
    "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsInC."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtDCInfo emlrtDCI = {
    89,                                          /* lineNo */
    22,                                          /* colNo */
    "FullStateDynamicsInC/fast_triangle_interp", /* fName */
    "C:"
    "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsInC."
    "m", /* pName */
    1    /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    1,                                           /* iFirst */
    1000,                                        /* iLast */
    89,                                          /* lineNo */
    34,                                          /* colNo */
    "Z_map",                                     /* aName */
    "FullStateDynamicsInC/fast_triangle_interp", /* fName */
    "C:"
    "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsInC."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = {
    89,                                          /* lineNo */
    34,                                          /* colNo */
    "FullStateDynamicsInC/fast_triangle_interp", /* fName */
    "C:"
    "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsInC."
    "m", /* pName */
    1    /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    1,                                           /* iFirst */
    1000,                                        /* iLast */
    90,                                          /* lineNo */
    34,                                          /* colNo */
    "Z_map",                                     /* aName */
    "FullStateDynamicsInC/fast_triangle_interp", /* fName */
    "C:"
    "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsInC."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = {
    90,                                          /* lineNo */
    34,                                          /* colNo */
    "FullStateDynamicsInC/fast_triangle_interp", /* fName */
    "C:"
    "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsInC."
    "m", /* pName */
    1    /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    1,                                           /* iFirst */
    1000,                                        /* iLast */
    91,                                          /* lineNo */
    22,                                          /* colNo */
    "Z_map",                                     /* aName */
    "FullStateDynamicsInC/fast_triangle_interp", /* fName */
    "C:"
    "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsInC."
    "m", /* pName */
    0    /* checkKind */
};

static emlrtDCInfo d_emlrtDCI = {
    91,                                          /* lineNo */
    22,                                          /* colNo */
    "FullStateDynamicsInC/fast_triangle_interp", /* fName */
    "C:"
    "\\Users\\srikr\\Desktop\\SPARX\\Matlab\\Controller\\FullStateDynamicsInC."
    "m", /* pName */
    1    /* checkKind */
};

/* Function Declarations */
static real_T att(const emlrtStack *sp, const c_captured_var *body_wheels,
                  const b_captured_var *Z_map, const captured_var *xmin,
                  const captured_var *ymin, const captured_var *dx,
                  const captured_var *dy, real_T xq, real_T yq, real_T yq_yaw,
                  real_T *pq);

/* Function Definitions */
static real_T att(const emlrtStack *sp, const c_captured_var *body_wheels,
                  const b_captured_var *Z_map, const captured_var *xmin,
                  const captured_var *ymin, const captured_var *dx,
                  const captured_var *dy, real_T xq, real_T yq, real_T yq_yaw,
                  real_T *pq)
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  real_T Aq[12];
  real_T At[12];
  real_T c_y[9];
  real_T wq[8];
  real_T c_Rq_tmp[4];
  real_T d_y[3];
  real_T b_xq[2];
  real_T Rq_tmp;
  real_T a21;
  real_T b_Rq_tmp;
  real_T b_dx;
  real_T b_dy;
  real_T b_is_upper_tmp;
  real_T b_xmin;
  real_T b_y;
  real_T b_ymin;
  real_T idx_x_forward;
  real_T idx_y_forward;
  real_T is_upper_tmp;
  real_T rq;
  real_T y;
  real_T zq_idx_0;
  real_T zq_idx_1;
  real_T zq_idx_2;
  real_T zq_idx_3;
  int32_T b_r1;
  int32_T b_r2;
  int32_T i;
  int32_T r3;
  boolean_T is_upper;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  /*  --- nested attitude helper --- */
  Rq_tmp = muDoubleScalarSin(yq_yaw);
  b_Rq_tmp = muDoubleScalarCos(yq_yaw);
  c_Rq_tmp[0] = b_Rq_tmp;
  c_Rq_tmp[2] = -Rq_tmp;
  c_Rq_tmp[1] = Rq_tmp;
  c_Rq_tmp[3] = b_Rq_tmp;
  memset(&wq[0], 0, sizeof(real_T) << 3);
  b_xq[0] = xq;
  b_xq[1] = yq;
  r = _mm_loadu_pd(&b_xq[0]);
  for (i = 0; i < 4; i++) {
    r1 = _mm_loadu_pd(&c_Rq_tmp[0]);
    b_r1 = i << 1;
    r2 = _mm_loadu_pd(&wq[b_r1]);
    _mm_storeu_pd(
        &wq[b_r1],
        _mm_add_pd(r2,
                   _mm_mul_pd(r1, _mm_set1_pd(body_wheels->contents[b_r1]))));
    r1 = _mm_loadu_pd(&c_Rq_tmp[2]);
    r2 = _mm_loadu_pd(&wq[b_r1]);
    _mm_storeu_pd(
        &wq[b_r1],
        _mm_add_pd(
            r2, _mm_mul_pd(r1, _mm_set1_pd(body_wheels->contents[b_r1 + 1]))));
    r1 = _mm_loadu_pd(&wq[b_r1]);
    _mm_storeu_pd(&wq[b_r1], _mm_add_pd(r1, r));
  }
  /*  zq = interp2(X_map, Y_map, Z_map, wq(1,:), wq(2,:), 'linear'); */
  b_xmin = xmin->contents;
  b_ymin = ymin->contents;
  b_dx = dx->contents;
  b_dy = dy->contents;
  st.site = &i_emlrtRSI;
  rq = (wq[0] - b_xmin) / b_dx;
  y = muDoubleScalarFloor(rq);
  a21 = (wq[1] - b_ymin) / b_dy;
  b_y = muDoubleScalarFloor(a21);
  zq_idx_3 = muDoubleScalarCeil(rq);
  idx_x_forward = zq_idx_3 + 1.0;
  rq = muDoubleScalarCeil(a21);
  idx_y_forward = rq + 1.0;
  /*  no integer bullshit */
  if (zq_idx_3 + 1.0 == y + 1.0) {
    idx_x_forward = (y + 1.0) + 1.0;
  }
  if (rq + 1.0 == b_y + 1.0) {
    idx_y_forward = (b_y + 1.0) + 1.0;
  }
  if (b_y + 1.0 != (int32_T)(b_y + 1.0)) {
    emlrtIntegerCheckR2012b(b_y + 1.0, &emlrtDCI, &st);
  }
  if (((int32_T)(b_y + 1.0) < 1) || ((int32_T)(b_y + 1.0) > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(b_y + 1.0), 1, 1000, &emlrtBCI,
                                  &st);
  }
  if (y + 1.0 != (int32_T)(y + 1.0)) {
    emlrtIntegerCheckR2012b(y + 1.0, &b_emlrtDCI, &st);
  }
  if (((int32_T)(y + 1.0) < 1) || ((int32_T)(y + 1.0) > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(y + 1.0), 1, 1000, &b_emlrtBCI,
                                  &st);
  }
  /*  Back-Back */
  if (idx_x_forward != (int32_T)idx_x_forward) {
    emlrtIntegerCheckR2012b(idx_x_forward, &c_emlrtDCI, &st);
  }
  if (((int32_T)idx_x_forward < 1) || ((int32_T)idx_x_forward > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)idx_x_forward, 1, 1000, &c_emlrtBCI,
                                  &st);
  }
  /*  Forward-Back  */
  if (idx_y_forward != (int32_T)idx_y_forward) {
    emlrtIntegerCheckR2012b(idx_y_forward, &d_emlrtDCI, &st);
  }
  if (((int32_T)idx_y_forward < 1) || ((int32_T)idx_y_forward > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)idx_y_forward, 1, 1000, &d_emlrtBCI,
                                  &st);
  }
  /*  Back-Forward */
  /*  Forward-Forward */
  /*  x,y values for these tings */
  /*  Check if past the diagonal */
  is_upper_tmp = wq[0] - (b_xmin + ((y + 1.0) - 1.0) * b_dx);
  b_is_upper_tmp = wq[1] - (b_ymin + ((b_y + 1.0) - 1.0) * b_dy);
  is_upper = (is_upper_tmp / b_dx + b_is_upper_tmp / b_dy > 1.0);
  /*  Calculate both planes (z = ax + by + c). */
  /*  a, b i sjust using basic slope formula */
  /*  ternary expression. */
  b_r1 = 1000 * ((int32_T)(y + 1.0) - 1);
  a21 = Z_map->contents[((int32_T)(b_y + 1.0) + b_r1) - 1];
  b_r2 = 1000 * ((int32_T)idx_x_forward - 1);
  zq_idx_3 = Z_map->contents[((int32_T)idx_y_forward + b_r1) - 1];
  y = Z_map->contents[((int32_T)idx_y_forward + b_r2) - 1];
  rq = Z_map->contents[((int32_T)(b_y + 1.0) + b_r2) - 1];
  zq_idx_0 = (real_T)!is_upper * (((rq - a21) / b_dx * is_upper_tmp +
                                   (zq_idx_3 - a21) / b_dy * b_is_upper_tmp) +
                                  a21) +
             (real_T)is_upper *
                 (((y - zq_idx_3) / b_dx *
                       (wq[0] - (b_xmin + (idx_x_forward - 1.0) * b_dx)) +
                   (y - rq) / b_dy *
                       (wq[1] - (b_ymin + (idx_y_forward - 1.0) * b_dy))) +
                  y);
  st.site = &i_emlrtRSI;
  rq = (wq[2] - b_xmin) / b_dx;
  y = muDoubleScalarFloor(rq);
  a21 = (wq[3] - b_ymin) / b_dy;
  b_y = muDoubleScalarFloor(a21);
  zq_idx_3 = muDoubleScalarCeil(rq);
  idx_x_forward = zq_idx_3 + 1.0;
  rq = muDoubleScalarCeil(a21);
  idx_y_forward = rq + 1.0;
  /*  no integer bullshit */
  if (zq_idx_3 + 1.0 == y + 1.0) {
    idx_x_forward = (y + 1.0) + 1.0;
  }
  if (rq + 1.0 == b_y + 1.0) {
    idx_y_forward = (b_y + 1.0) + 1.0;
  }
  if (b_y + 1.0 != (int32_T)(b_y + 1.0)) {
    emlrtIntegerCheckR2012b(b_y + 1.0, &emlrtDCI, &st);
  }
  if (((int32_T)(b_y + 1.0) < 1) || ((int32_T)(b_y + 1.0) > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(b_y + 1.0), 1, 1000, &emlrtBCI,
                                  &st);
  }
  if (y + 1.0 != (int32_T)(y + 1.0)) {
    emlrtIntegerCheckR2012b(y + 1.0, &b_emlrtDCI, &st);
  }
  if (((int32_T)(y + 1.0) < 1) || ((int32_T)(y + 1.0) > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(y + 1.0), 1, 1000, &b_emlrtBCI,
                                  &st);
  }
  /*  Back-Back */
  if (idx_x_forward != (int32_T)idx_x_forward) {
    emlrtIntegerCheckR2012b(idx_x_forward, &c_emlrtDCI, &st);
  }
  if (((int32_T)idx_x_forward < 1) || ((int32_T)idx_x_forward > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)idx_x_forward, 1, 1000, &c_emlrtBCI,
                                  &st);
  }
  /*  Forward-Back  */
  if (idx_y_forward != (int32_T)idx_y_forward) {
    emlrtIntegerCheckR2012b(idx_y_forward, &d_emlrtDCI, &st);
  }
  if (((int32_T)idx_y_forward < 1) || ((int32_T)idx_y_forward > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)idx_y_forward, 1, 1000, &d_emlrtBCI,
                                  &st);
  }
  /*  Back-Forward */
  /*  Forward-Forward */
  /*  x,y values for these tings */
  /*  Check if past the diagonal */
  is_upper_tmp = wq[2] - (b_xmin + ((y + 1.0) - 1.0) * b_dx);
  b_is_upper_tmp = wq[3] - (b_ymin + ((b_y + 1.0) - 1.0) * b_dy);
  is_upper = (is_upper_tmp / b_dx + b_is_upper_tmp / b_dy > 1.0);
  /*  Calculate both planes (z = ax + by + c). */
  /*  a, b i sjust using basic slope formula */
  /*  ternary expression. */
  b_r1 = 1000 * ((int32_T)(y + 1.0) - 1);
  a21 = Z_map->contents[((int32_T)(b_y + 1.0) + b_r1) - 1];
  b_r2 = 1000 * ((int32_T)idx_x_forward - 1);
  zq_idx_3 = Z_map->contents[((int32_T)idx_y_forward + b_r1) - 1];
  y = Z_map->contents[((int32_T)idx_y_forward + b_r2) - 1];
  rq = Z_map->contents[((int32_T)(b_y + 1.0) + b_r2) - 1];
  zq_idx_1 = (real_T)!is_upper * (((rq - a21) / b_dx * is_upper_tmp +
                                   (zq_idx_3 - a21) / b_dy * b_is_upper_tmp) +
                                  a21) +
             (real_T)is_upper *
                 (((y - zq_idx_3) / b_dx *
                       (wq[2] - (b_xmin + (idx_x_forward - 1.0) * b_dx)) +
                   (y - rq) / b_dy *
                       (wq[3] - (b_ymin + (idx_y_forward - 1.0) * b_dy))) +
                  y);
  st.site = &i_emlrtRSI;
  rq = (wq[4] - b_xmin) / b_dx;
  y = muDoubleScalarFloor(rq);
  a21 = (wq[5] - b_ymin) / b_dy;
  b_y = muDoubleScalarFloor(a21);
  zq_idx_3 = muDoubleScalarCeil(rq);
  idx_x_forward = zq_idx_3 + 1.0;
  rq = muDoubleScalarCeil(a21);
  idx_y_forward = rq + 1.0;
  /*  no integer bullshit */
  if (zq_idx_3 + 1.0 == y + 1.0) {
    idx_x_forward = (y + 1.0) + 1.0;
  }
  if (rq + 1.0 == b_y + 1.0) {
    idx_y_forward = (b_y + 1.0) + 1.0;
  }
  if (b_y + 1.0 != (int32_T)(b_y + 1.0)) {
    emlrtIntegerCheckR2012b(b_y + 1.0, &emlrtDCI, &st);
  }
  if (((int32_T)(b_y + 1.0) < 1) || ((int32_T)(b_y + 1.0) > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(b_y + 1.0), 1, 1000, &emlrtBCI,
                                  &st);
  }
  if (y + 1.0 != (int32_T)(y + 1.0)) {
    emlrtIntegerCheckR2012b(y + 1.0, &b_emlrtDCI, &st);
  }
  if (((int32_T)(y + 1.0) < 1) || ((int32_T)(y + 1.0) > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(y + 1.0), 1, 1000, &b_emlrtBCI,
                                  &st);
  }
  /*  Back-Back */
  if (idx_x_forward != (int32_T)idx_x_forward) {
    emlrtIntegerCheckR2012b(idx_x_forward, &c_emlrtDCI, &st);
  }
  if (((int32_T)idx_x_forward < 1) || ((int32_T)idx_x_forward > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)idx_x_forward, 1, 1000, &c_emlrtBCI,
                                  &st);
  }
  /*  Forward-Back  */
  if (idx_y_forward != (int32_T)idx_y_forward) {
    emlrtIntegerCheckR2012b(idx_y_forward, &d_emlrtDCI, &st);
  }
  if (((int32_T)idx_y_forward < 1) || ((int32_T)idx_y_forward > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)idx_y_forward, 1, 1000, &d_emlrtBCI,
                                  &st);
  }
  /*  Back-Forward */
  /*  Forward-Forward */
  /*  x,y values for these tings */
  /*  Check if past the diagonal */
  is_upper_tmp = wq[4] - (b_xmin + ((y + 1.0) - 1.0) * b_dx);
  b_is_upper_tmp = wq[5] - (b_ymin + ((b_y + 1.0) - 1.0) * b_dy);
  is_upper = (is_upper_tmp / b_dx + b_is_upper_tmp / b_dy > 1.0);
  /*  Calculate both planes (z = ax + by + c). */
  /*  a, b i sjust using basic slope formula */
  /*  ternary expression. */
  b_r1 = 1000 * ((int32_T)(y + 1.0) - 1);
  a21 = Z_map->contents[((int32_T)(b_y + 1.0) + b_r1) - 1];
  b_r2 = 1000 * ((int32_T)idx_x_forward - 1);
  zq_idx_3 = Z_map->contents[((int32_T)idx_y_forward + b_r1) - 1];
  y = Z_map->contents[((int32_T)idx_y_forward + b_r2) - 1];
  rq = Z_map->contents[((int32_T)(b_y + 1.0) + b_r2) - 1];
  zq_idx_2 = (real_T)!is_upper * (((rq - a21) / b_dx * is_upper_tmp +
                                   (zq_idx_3 - a21) / b_dy * b_is_upper_tmp) +
                                  a21) +
             (real_T)is_upper *
                 (((y - zq_idx_3) / b_dx *
                       (wq[4] - (b_xmin + (idx_x_forward - 1.0) * b_dx)) +
                   (y - rq) / b_dy *
                       (wq[5] - (b_ymin + (idx_y_forward - 1.0) * b_dy))) +
                  y);
  st.site = &i_emlrtRSI;
  rq = (wq[6] - b_xmin) / b_dx;
  y = muDoubleScalarFloor(rq);
  a21 = (wq[7] - b_ymin) / b_dy;
  b_y = muDoubleScalarFloor(a21);
  zq_idx_3 = muDoubleScalarCeil(rq);
  idx_x_forward = zq_idx_3 + 1.0;
  rq = muDoubleScalarCeil(a21);
  idx_y_forward = rq + 1.0;
  /*  no integer bullshit */
  if (zq_idx_3 + 1.0 == y + 1.0) {
    idx_x_forward = (y + 1.0) + 1.0;
  }
  if (rq + 1.0 == b_y + 1.0) {
    idx_y_forward = (b_y + 1.0) + 1.0;
  }
  if (b_y + 1.0 != (int32_T)(b_y + 1.0)) {
    emlrtIntegerCheckR2012b(b_y + 1.0, &emlrtDCI, &st);
  }
  if (((int32_T)(b_y + 1.0) < 1) || ((int32_T)(b_y + 1.0) > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(b_y + 1.0), 1, 1000, &emlrtBCI,
                                  &st);
  }
  if (y + 1.0 != (int32_T)(y + 1.0)) {
    emlrtIntegerCheckR2012b(y + 1.0, &b_emlrtDCI, &st);
  }
  if (((int32_T)(y + 1.0) < 1) || ((int32_T)(y + 1.0) > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(y + 1.0), 1, 1000, &b_emlrtBCI,
                                  &st);
  }
  /*  Back-Back */
  if (idx_x_forward != (int32_T)idx_x_forward) {
    emlrtIntegerCheckR2012b(idx_x_forward, &c_emlrtDCI, &st);
  }
  if (((int32_T)idx_x_forward < 1) || ((int32_T)idx_x_forward > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)idx_x_forward, 1, 1000, &c_emlrtBCI,
                                  &st);
  }
  /*  Forward-Back  */
  if (idx_y_forward != (int32_T)idx_y_forward) {
    emlrtIntegerCheckR2012b(idx_y_forward, &d_emlrtDCI, &st);
  }
  if (((int32_T)idx_y_forward < 1) || ((int32_T)idx_y_forward > 1000)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)idx_y_forward, 1, 1000, &d_emlrtBCI,
                                  &st);
  }
  /*  Back-Forward */
  /*  Forward-Forward */
  /*  x,y values for these tings */
  /*  Check if past the diagonal */
  is_upper_tmp = wq[6] - (b_xmin + ((y + 1.0) - 1.0) * b_dx);
  b_is_upper_tmp = wq[7] - (b_ymin + ((b_y + 1.0) - 1.0) * b_dy);
  is_upper = (is_upper_tmp / b_dx + b_is_upper_tmp / b_dy > 1.0);
  /*  Calculate both planes (z = ax + by + c). */
  /*  a, b i sjust using basic slope formula */
  /*  ternary expression. */
  b_r1 = 1000 * ((int32_T)(y + 1.0) - 1);
  a21 = Z_map->contents[((int32_T)(b_y + 1.0) + b_r1) - 1];
  b_r2 = 1000 * ((int32_T)idx_x_forward - 1);
  zq_idx_3 = Z_map->contents[((int32_T)idx_y_forward + b_r1) - 1];
  y = Z_map->contents[((int32_T)idx_y_forward + b_r2) - 1];
  rq = Z_map->contents[((int32_T)(b_y + 1.0) + b_r2) - 1];
  zq_idx_3 = (real_T)!is_upper * (((rq - a21) / b_dx * is_upper_tmp +
                                   (zq_idx_3 - a21) / b_dy * b_is_upper_tmp) +
                                  a21) +
             (real_T)is_upper *
                 (((y - zq_idx_3) / b_dx *
                       (wq[6] - (b_xmin + (idx_x_forward - 1.0) * b_dx)) +
                   (y - rq) / b_dy *
                       (wq[7] - (b_ymin + (idx_y_forward - 1.0) * b_dy))) +
                  y);
  /*  Replace abcq = Aq \ zq'; with this: */
  for (i = 0; i < 4; i++) {
    b_r1 = i << 1;
    rq = wq[b_r1];
    Aq[i] = rq;
    a21 = wq[b_r1 + 1];
    Aq[i + 4] = a21;
    Aq[i + 8] = 1.0;
    At[3 * i] = rq;
    At[3 * i + 1] = a21;
    At[3 * i + 2] = 1.0;
  }
  memset(&c_y[0], 0, 9U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    b_r1 = i << 2;
    rq = Aq[b_r1];
    r1 = _mm_loadu_pd(&At[0]);
    r2 = _mm_loadu_pd(&c_y[3 * i]);
    _mm_storeu_pd(&c_y[3 * i], _mm_add_pd(r2, _mm_mul_pd(r1, _mm_set1_pd(rq))));
    b_r2 = 3 * i + 2;
    c_y[b_r2] += At[2] * rq;
    rq = Aq[b_r1 + 1];
    r1 = _mm_loadu_pd(&At[3]);
    r2 = _mm_loadu_pd(&c_y[3 * i]);
    _mm_storeu_pd(&c_y[3 * i], _mm_add_pd(r2, _mm_mul_pd(r1, _mm_set1_pd(rq))));
    c_y[b_r2] += At[5] * rq;
    rq = Aq[b_r1 + 2];
    r1 = _mm_loadu_pd(&At[6]);
    r2 = _mm_loadu_pd(&c_y[3 * i]);
    _mm_storeu_pd(&c_y[3 * i], _mm_add_pd(r2, _mm_mul_pd(r1, _mm_set1_pd(rq))));
    c_y[b_r2] += At[8] * rq;
    rq = Aq[b_r1 + 3];
    r1 = _mm_loadu_pd(&At[9]);
    r2 = _mm_loadu_pd(&c_y[3 * i]);
    _mm_storeu_pd(&c_y[3 * i], _mm_add_pd(r2, _mm_mul_pd(r1, _mm_set1_pd(rq))));
    c_y[b_r2] += At[11] * rq;
  }
  memset(&d_y[0], 0, 3U * sizeof(real_T));
  r1 = _mm_loadu_pd(&At[0]);
  r2 = _mm_loadu_pd(&d_y[0]);
  _mm_storeu_pd(&d_y[0], _mm_add_pd(r2, _mm_mul_pd(r1, _mm_set1_pd(zq_idx_0))));
  d_y[2] += At[2] * zq_idx_0;
  r1 = _mm_loadu_pd(&At[3]);
  r2 = _mm_loadu_pd(&d_y[0]);
  _mm_storeu_pd(&d_y[0], _mm_add_pd(r2, _mm_mul_pd(r1, _mm_set1_pd(zq_idx_1))));
  d_y[2] += At[5] * zq_idx_1;
  r1 = _mm_loadu_pd(&At[6]);
  r2 = _mm_loadu_pd(&d_y[0]);
  _mm_storeu_pd(&d_y[0], _mm_add_pd(r2, _mm_mul_pd(r1, _mm_set1_pd(zq_idx_2))));
  d_y[2] += At[8] * zq_idx_2;
  r1 = _mm_loadu_pd(&At[9]);
  r2 = _mm_loadu_pd(&d_y[0]);
  _mm_storeu_pd(&d_y[0], _mm_add_pd(r2, _mm_mul_pd(r1, _mm_set1_pd(zq_idx_3))));
  d_y[2] += At[11] * zq_idx_3;
  st.site = &j_emlrtRSI;
  b_st.site = &k_emlrtRSI;
  c_st.site = &l_emlrtRSI;
  d_st.site = &m_emlrtRSI;
  b_r1 = 0;
  b_r2 = 1;
  r3 = 2;
  rq = muDoubleScalarAbs(c_y[0]);
  a21 = muDoubleScalarAbs(c_y[1]);
  if (a21 > rq) {
    rq = a21;
    b_r1 = 1;
    b_r2 = 0;
  }
  if (muDoubleScalarAbs(c_y[2]) > rq) {
    b_r1 = 2;
    b_r2 = 1;
    r3 = 0;
  }
  c_y[b_r2] /= c_y[b_r1];
  c_y[r3] /= c_y[b_r1];
  c_y[b_r2 + 3] -= c_y[b_r2] * c_y[b_r1 + 3];
  c_y[r3 + 3] -= c_y[r3] * c_y[b_r1 + 3];
  c_y[b_r2 + 6] -= c_y[b_r2] * c_y[b_r1 + 6];
  c_y[r3 + 6] -= c_y[r3] * c_y[b_r1 + 6];
  if (muDoubleScalarAbs(c_y[r3 + 3]) > muDoubleScalarAbs(c_y[b_r2 + 3])) {
    int32_T rtemp;
    rtemp = b_r2;
    b_r2 = r3;
    r3 = rtemp;
  }
  c_y[r3 + 3] /= c_y[b_r2 + 3];
  c_y[r3 + 6] -= c_y[r3 + 3] * c_y[b_r2 + 6];
  if ((c_y[b_r1] == 0.0) || (c_y[b_r2 + 3] == 0.0) || (c_y[r3 + 6] == 0.0)) {
    e_st.site = &n_emlrtRSI;
    if (!emlrtSetWarningFlag(&e_st)) {
      f_st.site = &o_emlrtRSI;
      warning(&f_st);
    }
  }
  a21 = d_y[b_r2] - d_y[b_r1] * c_y[b_r2];
  rq = ((d_y[r3] - d_y[b_r1] * c_y[r3]) - a21 * c_y[r3 + 3]) / c_y[r3 + 6];
  a21 -= rq * c_y[b_r2 + 6];
  a21 /= c_y[b_r2 + 3];
  rq = ((d_y[b_r1] - rq * c_y[b_r1 + 6]) - a21 * c_y[b_r1 + 3]) / c_y[b_r1];
  *pq = muDoubleScalarAtan(b_Rq_tmp * rq + Rq_tmp * a21);
  return muDoubleScalarAtan(-Rq_tmp * rq + b_Rq_tmp * a21);
}

void FullStateDynamicsInC(FullStateDynamicsInCStackData *SD,
                          const emlrtStack *sp, const real_T state[9],
                          const real_T u[4], real_T xmin, real_T ymin,
                          real_T dx, real_T dy, const real_T Z_map[1000000],
                          const struct0_T *geom, real_T dt, real_T dstate[9])
{
  c_captured_var body_wheels;
  captured_var b_dx;
  captured_var b_dy;
  captured_var b_xmin;
  captured_var b_ymin;
  emlrtStack st;
  real_T b_vx_tmp;
  real_T b_vy_tmp;
  real_T c_vx_tmp;
  real_T dpitch;
  real_T dv;
  real_T dyaw;
  real_T pitch_mp;
  real_T pitch_my;
  real_T pitch_pp;
  real_T pitch_py;
  real_T roll_mp;
  real_T roll_mx;
  real_T roll_my;
  real_T roll_pp;
  real_T roll_px;
  real_T roll_py;
  real_T v;
  real_T v_tmp;
  real_T vx;
  real_T vx_tmp;
  real_T vy;
  real_T vy_tmp;
  real_T vz_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  b_xmin.contents = xmin;
  b_ymin.contents = ymin;
  b_dx.contents = dx;
  b_dy.contents = dy;
  memcpy(&SD->f0.Z_map.contents[0], &Z_map[0], 1000000U * sizeof(real_T));
  /*  set mur, mul dependent on the dynamics */
  /*  mu_r = 0.8 + 0.2 * exp(-abs(w_r)); */
  /*  mu_l = 0.8 + 0.2 * exp(-abs(w_l)); */
  /*  --- diff-drive kinematics --- */
  v_tmp = geom->r / 2.0;
  v = v_tmp * (u[0] + u[1]);
  /*  Velocities */
  vx_tmp = muDoubleScalarCos(state[7]);
  b_vx_tmp = muDoubleScalarCos(state[8]);
  c_vx_tmp = v * b_vx_tmp;
  vx = c_vx_tmp * vx_tmp;
  vy_tmp = muDoubleScalarSin(state[8]);
  b_vy_tmp = v * vy_tmp;
  vy = b_vy_tmp * vx_tmp;
  vz_tmp = muDoubleScalarSin(state[7]);
  /*  Accelerations */
  dv = v_tmp * (u[2] + u[3]) / dt;
  /*  --- wheel positions & terrain query --- */
  v_tmp = geom->L / 2.0;
  body_wheels.contents[0] = v_tmp;
  body_wheels.contents[2] = v_tmp;
  v_tmp = -geom->L / 2.0;
  body_wheels.contents[4] = v_tmp;
  body_wheels.contents[6] = v_tmp;
  v_tmp = -geom->B / 2.0;
  body_wheels.contents[1] = v_tmp;
  dpitch = geom->B / 2.0;
  body_wheels.contents[3] = dpitch;
  body_wheels.contents[5] = v_tmp;
  body_wheels.contents[7] = dpitch;
  /*  --- attitude rates (spatial perturbation) --- */
  st.site = &b_emlrtRSI;
  roll_px = att(&st, &body_wheels, &SD->f0.Z_map, &b_xmin, &b_ymin, &b_dx,
                &b_dy, state[0] + 1.0E-6, state[1], state[8], &v_tmp);
  st.site = &c_emlrtRSI;
  roll_mx = att(&st, &body_wheels, &SD->f0.Z_map, &b_xmin, &b_ymin, &b_dx,
                &b_dy, state[0] - 1.0E-6, state[1], state[8], &dpitch);
  st.site = &d_emlrtRSI;
  roll_py = att(&st, &body_wheels, &SD->f0.Z_map, &b_xmin, &b_ymin, &b_dx,
                &b_dy, state[0], state[1] + 1.0E-6, state[8], &pitch_py);
  st.site = &e_emlrtRSI;
  roll_my = att(&st, &body_wheels, &SD->f0.Z_map, &b_xmin, &b_ymin, &b_dx,
                &b_dy, state[0], state[1] - 1.0E-6, state[8], &pitch_my);
  st.site = &f_emlrtRSI;
  roll_pp = att(&st, &body_wheels, &SD->f0.Z_map, &b_xmin, &b_ymin, &b_dx,
                &b_dy, state[0], state[1], state[8] + 1.0E-6, &pitch_pp);
  st.site = &g_emlrtRSI;
  roll_mp = att(&st, &body_wheels, &SD->f0.Z_map, &b_xmin, &b_ymin, &b_dx,
                &b_dy, state[0], state[1], state[8] - 1.0E-6, &pitch_mp);
  dyaw = geom->r / geom->B * (u[0] - u[1]);
  dpitch =
      ((v_tmp - dpitch) / 2.0E-6 * vx + (pitch_py - pitch_my) / 2.0E-6 * vy) +
      (pitch_pp - pitch_mp) / 2.0E-6 * dyaw;
  /*  --- accelerations (u constant within step) --- */
  /*  --- pack --- */
  dstate[0] = vx;
  dstate[1] = vy;
  dstate[2] = v * vz_tmp;
  v_tmp = v * vx_tmp;
  dstate[3] = (b_vx_tmp * vx_tmp * dv - v_tmp * vy_tmp * dyaw) -
              c_vx_tmp * vz_tmp * dpitch;
  dstate[4] = (vy_tmp * vx_tmp * dv + v_tmp * b_vx_tmp * dyaw) -
              b_vy_tmp * vz_tmp * dpitch;
  dstate[5] = vz_tmp * dv + v_tmp * dpitch;
  dstate[6] =
      ((roll_px - roll_mx) / 2.0E-6 * vx + (roll_py - roll_my) / 2.0E-6 * vy) +
      (roll_pp - roll_mp) / 2.0E-6 * dyaw;
  dstate[7] = dpitch;
  dstate[8] = dyaw;
}

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

/* End of code generation (FullStateDynamicsInC.c) */

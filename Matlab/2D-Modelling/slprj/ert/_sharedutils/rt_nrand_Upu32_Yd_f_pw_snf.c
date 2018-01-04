/*
 * File: rt_nrand_Upu32_Yd_f_pw_snf.c
 *
 * Code generated for Simulink model 'Sim_NL_XZ'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
 * C/C++ source code generated on : Thu Dec 14 10:46:42 2017
 */

#include "rtwtypes.h"
#include <math.h>
#include "rt_urand_Upu32_Yd_f_pw_snf.h"
#include "rt_nrand_Upu32_Yd_f_pw_snf.h"

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T y;
  real_T sr;
  real_T si;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

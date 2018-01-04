/*
 * File: Torque_Transformation.c
 *
 * Code generated for Simulink model 'Torque_Transformation'.
 *
 * Model version                  : 1.5
 * Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
 * C/C++ source code generated on : Thu Dec 14 10:47:55 2017
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Torque_Transformation.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        (*((rtm)->errorStatus))
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   (*((rtm)->errorStatus) = (val))
#endif

#ifndef rtmGetErrorStatusPointer
# define rtmGetErrorStatusPointer(rtm) (rtm)->errorStatus
#endif

#ifndef rtmSetErrorStatusPointer
# define rtmSetErrorStatusPointer(rtm, val) ((rtm)->errorStatus = (val))
#endif

/* Output and update for referenced model: 'Torque_Transformation' */
void Torque_Transformation(const real_T *rtu_T_x, const real_T *rtu_T_y, const
  real_T *rtu_T_z, real_T *rty_T1, real_T *rty_T2, real_T *rty_T3)
{
  /* Gain: '<Root>/Gain12' incorporates:
   *  Gain: '<Root>/Gain13'
   *  Gain: '<Root>/Gain14'
   *  Gain: '<Root>/Gain15'
   *  Gain: '<Root>/Gain16'
   *  Gain: '<Root>/Gain9'
   *  Sum: '<Root>/Sum6'
   *  Sum: '<Root>/Sum7'
   *  Sum: '<Root>/Sum8'
   *  Sum: '<Root>/Sum9'
   */
  *rty_T3 = (((1.7320508075688772 * *rtu_T_y - *rtu_T_x) * -0.49999999999999978
              + (1.7320508075688772 * *rtu_T_x + *rtu_T_y) * 0.86602540378443871)
             * 1.4142135623730949 + *rtu_T_z) * 0.33333333333333331;

  /* Gain: '<Root>/Gain5' incorporates:
   *  Gain: '<Root>/Gain'
   *  Gain: '<Root>/Gain1'
   *  Gain: '<Root>/Gain4'
   *  Sum: '<Root>/Sum'
   *  Sum: '<Root>/Sum1'
   */
  *rty_T1 = ((-0.49999999999999978 * *rtu_T_x - 0.86602540378443871 * *rtu_T_y) *
             2.82842712474619 + *rtu_T_z) * 0.33333333333333331;

  /* Gain: '<Root>/Gain7' incorporates:
   *  Gain: '<Root>/Gain10'
   *  Gain: '<Root>/Gain11'
   *  Gain: '<Root>/Gain2'
   *  Gain: '<Root>/Gain6'
   *  Gain: '<Root>/Gain8'
   *  Sum: '<Root>/Sum2'
   *  Sum: '<Root>/Sum3'
   *  Sum: '<Root>/Sum4'
   *  Sum: '<Root>/Sum5'
   */
  *rty_T2 = (((-1.7320508075688772 * *rtu_T_x + *rtu_T_y) * 0.86602540378443871
              - (1.7320508075688772 * *rtu_T_y + *rtu_T_x) *
              -0.49999999999999978) * 1.4142135623730949 + *rtu_T_z) *
    0.33333333333333331;
}

/* Model initialize function */
void Torque_Transformatio_initialize(const char_T **rt_errorStatus,
  RT_MODEL_Torque_Transformatio_T *const Torque_Transformation_M)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatusPointer(Torque_Transformation_M, rt_errorStatus);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

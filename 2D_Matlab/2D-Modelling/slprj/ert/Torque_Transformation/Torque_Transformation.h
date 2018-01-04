/*
 * File: Torque_Transformation.h
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

#ifndef RTW_HEADER_Torque_Transformation_h_
#define RTW_HEADER_Torque_Transformation_h_
#ifndef Torque_Transformation_COMMON_INCLUDES_
# define Torque_Transformation_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* Torque_Transformation_COMMON_INCLUDES_ */

/* Shared type includes */
#include "model_reference_types.h"

/* Forward declaration for rtModel */
typedef struct tag_RTM_Torque_Transformation_T RT_MODEL_Torque_Transformatio_T;

/* Real-time Model Data Structure */
struct tag_RTM_Torque_Transformation_T {
  const char_T **errorStatus;
};

typedef struct {
  RT_MODEL_Torque_Transformatio_T rtm;
} MdlrefDW_Torque_Transformatio_T;

/* Model reference registration function */
extern void Torque_Transformatio_initialize(const char_T **rt_errorStatus,
  RT_MODEL_Torque_Transformatio_T *const Torque_Transformation_M);
extern void Torque_Transformation(const real_T *rtu_T_x, const real_T *rtu_T_y,
  const real_T *rtu_T_z, real_T *rty_T1, real_T *rty_T2, real_T *rty_T3);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Torque_Transformation'
 */
#endif                                 /* RTW_HEADER_Torque_Transformation_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

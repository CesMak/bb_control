/*
 * File: Sim_NL_XZ.h
 *
 * Code generated for Simulink model 'Sim_NL_XZ'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
 * C/C++ source code generated on : Thu Dec 14 10:46:42 2017
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Sim_NL_XZ_h_
#define RTW_HEADER_Sim_NL_XZ_h_
#include <math.h>
#ifndef Sim_NL_XZ_COMMON_INCLUDES_
# define Sim_NL_XZ_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* Sim_NL_XZ_COMMON_INCLUDES_ */

/* Shared type includes */
#include "model_reference_types.h"
#include "rt_nrand_Upu32_Yd_f_pw_snf.h"

/* Forward declaration for rtModel */
typedef struct tag_RTM_Sim_NL_XZ_T RT_MODEL_Sim_NL_XZ_T;

/* Block signals and states (auto storage) for model 'Sim_NL_XZ' */
typedef struct {
  real_T dx[4];                        /* '<Root>/MATLAB Function' */
  real_T Output;                       /* '<S1>/Output' */
  real_T Sum;                          /* '<Root>/Sum' */
  real_T NextOutput;                   /* '<S1>/White Noise' */
  uint32_T RandSeed;                   /* '<S1>/White Noise' */
} DW_Sim_NL_XZ_f_T;

/* Continuous states for model 'Sim_NL_XZ' */
typedef struct {
  real_T Integrator_CSTATE[4];         /* '<Root>/Integrator' */
} X_Sim_NL_XZ_n_T;

/* State derivatives for model 'Sim_NL_XZ' */
typedef struct {
  real_T Integrator_CSTATE[4];         /* '<Root>/Integrator' */
} XDot_Sim_NL_XZ_n_T;

/* State Disabled for model 'Sim_NL_XZ' */
typedef struct {
  boolean_T Integrator_CSTATE[4];      /* '<Root>/Integrator' */
} XDis_Sim_NL_XZ_n_T;

/* Real-time Model Data Structure */
struct tag_RTM_Sim_NL_XZ_T {
  const char_T **errorStatus;
  RTWSolverInfo *solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T stepSize0;
    SimTimeStep *simTimeStep;
    boolean_T *stopRequestedFlag;
  } Timing;
};

typedef struct {
  DW_Sim_NL_XZ_f_T rtdw;
  RT_MODEL_Sim_NL_XZ_T rtm;
} MdlrefDW_Sim_NL_XZ_T;

/* Model reference registration function */
extern void Sim_NL_XZ_initialize(const char_T **rt_errorStatus, boolean_T
  *rt_stopRequested, RTWSolverInfo *rt_solverInfo, RT_MODEL_Sim_NL_XZ_T *const
  Sim_NL_XZ_M);
extern void Sim_NL_XZ_Init(DW_Sim_NL_XZ_f_T *localDW, X_Sim_NL_XZ_n_T *localX);
extern void Sim_NL_XZ_Deriv(DW_Sim_NL_XZ_f_T *localDW, XDot_Sim_NL_XZ_n_T
  *localXdot);
extern void Sim_NL_XZ_Update(RT_MODEL_Sim_NL_XZ_T * const Sim_NL_XZ_M,
  DW_Sim_NL_XZ_f_T *localDW);
extern void Sim_NL_XZ(RT_MODEL_Sim_NL_XZ_T * const Sim_NL_XZ_M, const real_T
                      *rtu_T_y, real_T *rty_phi_y, real_T *rty_theta_y, real_T
                      *rty_phi_y_dot, real_T *rty_theta_y_dot, real_T rty_x[4],
                      DW_Sim_NL_XZ_f_T *localDW, X_Sim_NL_XZ_n_T *localX);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/phi_y_dot_plot' : Unused code path elimination
 * Block '<Root>/phi_y_plot' : Unused code path elimination
 * Block '<Root>/theta_y_dot_plot' : Unused code path elimination
 * Block '<Root>/theta_y_plot' : Unused code path elimination
 */

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
 * '<Root>' : 'Sim_NL_XZ'
 * '<S1>'   : 'Sim_NL_XZ/Band-Limited White Noise'
 * '<S2>'   : 'Sim_NL_XZ/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_Sim_NL_XZ_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

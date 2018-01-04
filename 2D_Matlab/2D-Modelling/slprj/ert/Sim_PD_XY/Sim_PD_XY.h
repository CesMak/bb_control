/*
 * File: Sim_PD_XY.h
 *
 * Code generated for Simulink model 'Sim_PD_XY'.
 *
 * Model version                  : 1.12
 * Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
 * C/C++ source code generated on : Thu Dec 14 10:47:40 2017
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Sim_PD_XY_h_
#define RTW_HEADER_Sim_PD_XY_h_
#ifndef Sim_PD_XY_COMMON_INCLUDES_
# define Sim_PD_XY_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* Sim_PD_XY_COMMON_INCLUDES_ */

/* Shared type includes */
#include "model_reference_types.h"
#include "rt_nrand_Upu32_Yd_f_pw_snf.h"

/* Forward declaration for rtModel */
typedef struct tag_RTM_Sim_PD_XY_T RT_MODEL_Sim_PD_XY_T;

/* Block signals and states (auto storage) for model 'Sim_PD_XY' */
typedef struct {
  real_T Output;                       /* '<S1>/Output' */
  real_T Sum;                          /* '<Root>/Sum' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T Sum1;                         /* '<Root>/Sum1' */
  real_T NextOutput;                   /* '<S1>/White Noise' */
  uint32_T RandSeed;                   /* '<S1>/White Noise' */
} DW_Sim_PD_XY_f_T;

/* Continuous states for model 'Sim_PD_XY' */
typedef struct {
  real_T Integrator1_CSTATE;           /* '<Root>/Integrator1' */
  real_T TransferFcn1_CSTATE;          /* '<Root>/Transfer Fcn1' */
  real_T Integrator_CSTATE;            /* '<Root>/Integrator' */
} X_Sim_PD_XY_n_T;

/* State derivatives for model 'Sim_PD_XY' */
typedef struct {
  real_T Integrator1_CSTATE;           /* '<Root>/Integrator1' */
  real_T TransferFcn1_CSTATE;          /* '<Root>/Transfer Fcn1' */
  real_T Integrator_CSTATE;            /* '<Root>/Integrator' */
} XDot_Sim_PD_XY_n_T;

/* State Disabled for model 'Sim_PD_XY' */
typedef struct {
  boolean_T Integrator1_CSTATE;        /* '<Root>/Integrator1' */
  boolean_T TransferFcn1_CSTATE;       /* '<Root>/Transfer Fcn1' */
  boolean_T Integrator_CSTATE;         /* '<Root>/Integrator' */
} XDis_Sim_PD_XY_n_T;

/* Real-time Model Data Structure */
struct tag_RTM_Sim_PD_XY_T {
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
  DW_Sim_PD_XY_f_T rtdw;
  RT_MODEL_Sim_PD_XY_T rtm;
} MdlrefDW_Sim_PD_XY_T;

/* Model reference registration function */
extern void Sim_PD_XY_initialize(const char_T **rt_errorStatus, boolean_T
  *rt_stopRequested, RTWSolverInfo *rt_solverInfo, RT_MODEL_Sim_PD_XY_T *const
  Sim_PD_XY_M);
extern void Sim_PD_XY_Init(DW_Sim_PD_XY_f_T *localDW, X_Sim_PD_XY_n_T *localX);
extern void Sim_PD_XY_Deriv(DW_Sim_PD_XY_f_T *localDW, X_Sim_PD_XY_n_T *localX,
  XDot_Sim_PD_XY_n_T *localXdot);
extern void Sim_PD_XY_Update(RT_MODEL_Sim_PD_XY_T * const Sim_PD_XY_M,
  DW_Sim_PD_XY_f_T *localDW);
extern void Sim_PD_XY(RT_MODEL_Sim_PD_XY_T * const Sim_PD_XY_M, const real_T
                      *rtu_theta_z_AP, real_T *rty_theta_z, real_T *rty_T_z,
                      DW_Sim_PD_XY_f_T *localDW, X_Sim_PD_XY_n_T *localX);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/theta_z_plot' : Unused code path elimination
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
 * '<Root>' : 'Sim_PD_XY'
 * '<S1>'   : 'Sim_PD_XY/Band-Limited White Noise'
 */
#endif                                 /* RTW_HEADER_Sim_PD_XY_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

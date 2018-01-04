/*
 * File: Sim_PD_XY.c
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

#include "Sim_PD_XY.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       ((rtmGetSimTimeStep((rtm))) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       ((rtmGetSimTimeStep((rtm))) == MINOR_TIME_STEP)
#endif

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

#ifndef rtmGetSimTimeStep
# define rtmGetSimTimeStep(rtm)        (*((rtm)->Timing.simTimeStep))
#endif

#ifndef rtmGetSimTimeStepPointer
# define rtmGetSimTimeStepPointer(rtm) (rtm)->Timing.simTimeStep
#endif

#ifndef rtmSetSimTimeStepPointer
# define rtmSetSimTimeStepPointer(rtm, val) ((rtm)->Timing.simTimeStep = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      (*((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) (*((rtm)->Timing.stopRequestedFlag) = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequestedPtr
# define rtmSetStopRequestedPtr(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

/* System initialize for referenced model: 'Sim_PD_XY' */
void Sim_PD_XY_Init(DW_Sim_PD_XY_f_T *localDW, X_Sim_PD_XY_n_T *localX)
{
  /* InitializeConditions for RandomNumber: '<S1>/White Noise' */
  localDW->RandSeed = 1529675776U;
  localDW->NextOutput = rt_nrand_Upu32_Yd_f_pw_snf(&localDW->RandSeed);

  /* InitializeConditions for Integrator: '<Root>/Integrator1' */
  localX->Integrator1_CSTATE = 0.098174770424681035;

  /* InitializeConditions for TransferFcn: '<Root>/Transfer Fcn1' */
  localX->TransferFcn1_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<Root>/Integrator' */
  localX->Integrator_CSTATE = 0.0;
}

/* Outputs for referenced model: 'Sim_PD_XY' */
void Sim_PD_XY(RT_MODEL_Sim_PD_XY_T * const Sim_PD_XY_M, const real_T
               *rtu_theta_z_AP, real_T *rty_theta_z, real_T *rty_T_z,
               DW_Sim_PD_XY_f_T *localDW, X_Sim_PD_XY_n_T *localX)
{
  if (rtmIsMajorTimeStep(Sim_PD_XY_M)) {
    /* Gain: '<S1>/Output' incorporates:
     *  RandomNumber: '<S1>/White Noise'
     */
    localDW->Output = 0.95916630466254393 * localDW->NextOutput;
  }

  /* Integrator: '<Root>/Integrator1' */
  *rty_theta_z = localX->Integrator1_CSTATE;

  /* Sum: '<Root>/Sum' */
  localDW->Sum = *rtu_theta_z_AP - *rty_theta_z;

  /* TransferFcn: '<Root>/Transfer Fcn1' */
  *rty_T_z = 0.0;
  *rty_T_z += -268.2 * localX->TransferFcn1_CSTATE;
  *rty_T_z += 29.8 * localDW->Sum;

  /* Gain: '<Root>/Gain' */
  localDW->Gain = 1.0478784586011898 * *rty_T_z;

  /* Sum: '<Root>/Sum1' incorporates:
   *  Integrator: '<Root>/Integrator'
   */
  localDW->Sum1 = localX->Integrator_CSTATE + localDW->Output;
}

/* Update for referenced model: 'Sim_PD_XY' */
void Sim_PD_XY_Update(RT_MODEL_Sim_PD_XY_T * const Sim_PD_XY_M, DW_Sim_PD_XY_f_T
                      *localDW)
{
  if (rtmIsMajorTimeStep(Sim_PD_XY_M)) {
    /* Update for RandomNumber: '<S1>/White Noise' */
    localDW->NextOutput = rt_nrand_Upu32_Yd_f_pw_snf(&localDW->RandSeed);
  }
}

/* Derivatives for referenced model: 'Sim_PD_XY' */
void Sim_PD_XY_Deriv(DW_Sim_PD_XY_f_T *localDW, X_Sim_PD_XY_n_T *localX,
                     XDot_Sim_PD_XY_n_T *localXdot)
{
  /* Derivatives for Integrator: '<Root>/Integrator1' */
  localXdot->Integrator1_CSTATE = localDW->Sum1;

  /* Derivatives for TransferFcn: '<Root>/Transfer Fcn1' */
  localXdot->TransferFcn1_CSTATE = 0.0;
  localXdot->TransferFcn1_CSTATE += -10.0 * localX->TransferFcn1_CSTATE;
  localXdot->TransferFcn1_CSTATE += localDW->Sum;

  /* Derivatives for Integrator: '<Root>/Integrator' */
  localXdot->Integrator_CSTATE = localDW->Gain;
}

/* Model initialize function */
void Sim_PD_XY_initialize(const char_T **rt_errorStatus, boolean_T
  *rt_stopRequested, RTWSolverInfo *rt_solverInfo, RT_MODEL_Sim_PD_XY_T *const
  Sim_PD_XY_M)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatusPointer(Sim_PD_XY_M, rt_errorStatus);

  /* initialize stop requested flag */
  rtmSetStopRequestedPtr(Sim_PD_XY_M, rt_stopRequested);

  /* initialize RTWSolverInfo */
  Sim_PD_XY_M->solverInfo = (rt_solverInfo);

  /* Set the Timing fields to the appropriate data in the RTWSolverInfo */
  rtmSetSimTimeStepPointer(Sim_PD_XY_M, rtsiGetSimTimeStepPtr
    (Sim_PD_XY_M->solverInfo));
  Sim_PD_XY_M->Timing.stepSize0 = (rtsiGetStepSize(Sim_PD_XY_M->solverInfo));
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

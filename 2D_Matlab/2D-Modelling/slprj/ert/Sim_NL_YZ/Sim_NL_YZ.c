/*
 * File: Sim_NL_YZ.c
 *
 * Code generated for Simulink model 'Sim_NL_YZ'.
 *
 * Model version                  : 1.30
 * Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
 * C/C++ source code generated on : Thu Dec 14 10:47:17 2017
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Sim_NL_YZ.h"

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

/* System initialize for referenced model: 'Sim_NL_YZ' */
void Sim_NL_YZ_Init(DW_Sim_NL_YZ_f_T *localDW, X_Sim_NL_YZ_n_T *localX)
{
  /* InitializeConditions for RandomNumber: '<S1>/White Noise' */
  localDW->RandSeed = 1529675776U;
  localDW->NextOutput = rt_nrand_Upu32_Yd_f_pw_snf(&localDW->RandSeed);

  /* InitializeConditions for Integrator: '<Root>/Integrator' */
  localX->Integrator_CSTATE[0] = 0.0;
  localX->Integrator_CSTATE[1] = 0.098174770424681035;
  localX->Integrator_CSTATE[2] = 0.0;
  localX->Integrator_CSTATE[3] = 0.0;
}

/* Outputs for referenced model: 'Sim_NL_YZ' */
void Sim_NL_YZ(RT_MODEL_Sim_NL_YZ_T * const Sim_NL_YZ_M, const real_T *rtu_T_x,
               real_T *rty_phi_x, real_T *rty_theta_x, real_T *rty_phi_x_dot,
               real_T *rty_theta_x_dot, real_T rty_x[4], DW_Sim_NL_YZ_f_T
               *localDW, X_Sim_NL_YZ_n_T *localX)
{
  real_T x;
  real_T b_x;
  if (rtmIsMajorTimeStep(Sim_NL_YZ_M)) {
    /* Gain: '<S1>/Output' incorporates:
     *  RandomNumber: '<S1>/White Noise'
     */
    localDW->Output = 0.95916630466254393 * localDW->NextOutput;
  }

  /* Integrator: '<Root>/Integrator' */
  rty_x[0] = localX->Integrator_CSTATE[0];
  rty_x[1] = localX->Integrator_CSTATE[1];
  rty_x[2] = localX->Integrator_CSTATE[2];
  rty_x[3] = localX->Integrator_CSTATE[3];

  /* MATLAB Function: '<Root>/MATLAB Function' */
  /* Zuweisung Koordinaten und Ableitungen */
  /* MATLAB Function 'MATLAB Function': '<S2>:1' */
  /* '<S2>:1:4' phi_x =x(1); */
  /* '<S2>:1:5' theta_x=x(2); */
  /* '<S2>:1:6' phi_x_dot=x(3); */
  /* '<S2>:1:7' theta_x_dot=x(4); */
  /* '<S2>:1:9' phi_x_dotdot = (0.0355556*(1.65925e33*sin(theta_x)*theta_x_dot^2 + 5.804e34*T_x + 1.26585e35*sin(theta_x) - 8.4019e33*cos(theta_x)*sin(theta_x) + 4.60272e33*T_x*cos(theta_x)))/(6.42314e31*cos(theta_x) - 2.13164e30*cos(theta_x)^2 + 1.63981e32); */
  x = cos(rty_x[1]);

  /* '<S2>:1:11' theta_x_dotdot = (0.111111*(6.01665e33*T_x + 2.95242e34*sin(theta_x) + 2.89041e32*theta_x_dot^2*sin(theta_x) - 1.47287e33*T_x*cos(theta_x) - 1.91848e31*theta_x_dot^2*cos(theta_x)*sin(theta_x)))/(6.42314e31*cos(theta_x) - 2.13164e30*cos(theta_x)^2 + 1.63981e32); */
  b_x = cos(rty_x[1]);

  /* Zuweisung Ausgang */
  /* '<S2>:1:14' dx = [phi_x_dot; theta_x_dot; phi_x_dotdot; theta_x_dotdot]; */
  localDW->dx[0] = rty_x[2];
  localDW->dx[1] = rty_x[3];
  localDW->dx[2] = ((((1.65925E+33 * sin(rty_x[1]) * (rty_x[3] * rty_x[3]) +
                       5.804E+34 * *rtu_T_x) + 1.26585E+35 * sin(rty_x[1])) -
                     8.4019E+33 * cos(rty_x[1]) * sin(rty_x[1])) + 4.60272E+33 *
                    *rtu_T_x * cos(rty_x[1])) * 0.0355556 / ((6.42314E+31 * cos
    (rty_x[1]) - x * x * 2.13164E+30) + 1.63981E+32);
  localDW->dx[3] = (((rty_x[3] * rty_x[3] * 2.89041E+32 * sin(rty_x[1]) +
                      (6.01665E+33 * *rtu_T_x + 2.95242E+34 * sin(rty_x[1]))) -
                     1.47287E+33 * *rtu_T_x * cos(rty_x[1])) - rty_x[3] * rty_x
                    [3] * 1.91848E+31 * cos(rty_x[1]) * sin(rty_x[1])) *
    0.111111 / ((6.42314E+31 * cos(rty_x[1]) - b_x * b_x * 2.13164E+30) +
                1.63981E+32);

  /* Sum: '<Root>/Sum' */
  localDW->Sum = localDW->Output + localDW->dx[1];

  /* SignalConversion: '<Root>/TmpSignal ConversionAtphi_xInport1' */
  *rty_phi_x = rty_x[0];

  /* SignalConversion: '<Root>/TmpSignal ConversionAtphi_x_dotInport1' */
  *rty_phi_x_dot = rty_x[2];

  /* SignalConversion: '<Root>/TmpSignal ConversionAttheta_xInport1' */
  *rty_theta_x = rty_x[1];

  /* SignalConversion: '<Root>/TmpSignal ConversionAttheta_x_dotInport1' */
  *rty_theta_x_dot = rty_x[3];
}

/* Update for referenced model: 'Sim_NL_YZ' */
void Sim_NL_YZ_Update(RT_MODEL_Sim_NL_YZ_T * const Sim_NL_YZ_M, DW_Sim_NL_YZ_f_T
                      *localDW)
{
  if (rtmIsMajorTimeStep(Sim_NL_YZ_M)) {
    /* Update for RandomNumber: '<S1>/White Noise' */
    localDW->NextOutput = rt_nrand_Upu32_Yd_f_pw_snf(&localDW->RandSeed);
  }
}

/* Derivatives for referenced model: 'Sim_NL_YZ' */
void Sim_NL_YZ_Deriv(DW_Sim_NL_YZ_f_T *localDW, XDot_Sim_NL_YZ_n_T *localXdot)
{
  /* Derivatives for Integrator: '<Root>/Integrator' */
  localXdot->Integrator_CSTATE[0] = localDW->dx[0];
  localXdot->Integrator_CSTATE[1] = localDW->Sum;
  localXdot->Integrator_CSTATE[2] = localDW->dx[2];
  localXdot->Integrator_CSTATE[3] = localDW->dx[3];
}

/* Model initialize function */
void Sim_NL_YZ_initialize(const char_T **rt_errorStatus, boolean_T
  *rt_stopRequested, RTWSolverInfo *rt_solverInfo, RT_MODEL_Sim_NL_YZ_T *const
  Sim_NL_YZ_M)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatusPointer(Sim_NL_YZ_M, rt_errorStatus);

  /* initialize stop requested flag */
  rtmSetStopRequestedPtr(Sim_NL_YZ_M, rt_stopRequested);

  /* initialize RTWSolverInfo */
  Sim_NL_YZ_M->solverInfo = (rt_solverInfo);

  /* Set the Timing fields to the appropriate data in the RTWSolverInfo */
  rtmSetSimTimeStepPointer(Sim_NL_YZ_M, rtsiGetSimTimeStepPtr
    (Sim_NL_YZ_M->solverInfo));
  Sim_NL_YZ_M->Timing.stepSize0 = (rtsiGetStepSize(Sim_NL_YZ_M->solverInfo));
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Define.h>
#include <IMU.h>
#include <imu_spi.h>
#include <MadgwickAHRS.h>
#include <MPU9250.h>
#include <MPU9250_REGS.h>

#include "ballbot_motor_driver.h"

#define SAMPLE_TIME  40000      // in microseconds
#define T1          SAMPLE_TIME/2
#define TR          10*T1
#define K_PD        10       

#define B0          (TR/SAMPLE_TIME)*K_PD
#define B1          (-1-(2*TR/SAMPLE_TIME))*K_PD
#define B2          (1+TR/SAMPLE_TIME)*K_PD

//#define K_MOTOR    658.3092  // Current unit per Nm
#define K_EXP        7 // torque to unit factor 11.11 (Michi)

#define ALPHA       PI/4
#define BETA        PI/3
#define RK          0.07
#define RW          0.03

#define FAKT        0.7
#define X_OFFSET_RAD 0.05
#define Y_OFFSET_RAD -0.01

//#define DEBUG_SEN
#define PRINT_Values
//#define DEBUG_ANGLE
//#define DEBUG_VELOCITY
//#define DEBUG_PSI
//#define DEBUG_PSI_DOT

struct sensor_values{

  //x-Axis
  float phi_x_spoint ;
  float theta_x_spoint ;
  float psi_x_spoint ;

  float phi_x_cpoint ;
  float theta_x_cpoint;
  float psi_x_cpoint ;

  float phi_x_dot_spoint;
  float theta_x_dot_spoint;
  float psi_x_dot_spoint; 

  float phi_x_dot_cpoint;
  float theta_x_dot_cpoint; 
  float psi_x_dot_cpoint; 

  //xz planar
  float phi_y_spoint;
  float theta_y_spoint;
  float psi_y_spoint;

  float phi_y_cpoint;
  float theta_y_cpoint;
  float psi_y_cpoint;

  float phi_y_dot_spoint;
  float theta_y_dot_spoint;
  float psi_y_dot_spoint; 

  float phi_y_dot_cpoint;
  float theta_y_dot_cpoint; 
  float psi_y_dot_cpoint; 

  //xy planar
  float phi_z_spoint;
  float theta_z_spoint;
  float psi_z_spoint;

  float phi_z_cpoint;
  float theta_z_cpoint;
  float psi_z_cpoint;

  float phi_z_dot_spoint;
  float theta_z_dot_spoint;
  float psi_z_dot_spoint; 

  float phi_z_dot_cpoint;
  float theta_z_dot_cpoint; 
  float psi_z_dot_cpoint; 

};


struct controller_values
{
  // K_values for x-direction
  float K_yz_phi; 
  float K_yz_theta; 
  float K_yz_phi_dot; 
  float K_yz_theta_dot;

  // K_values for y-direction
  float K_xz_phi;
  float K_xz_theta;
  float K_xz_phi_dot;
  float K_xz_theta_dot;

  // controller values for z-direction
  float K_xy;
  float T_1;
  float T_r;

  // .....
  float diff;
  float e;
  float e_alt;

  //Actuator Size
  float u;
};


class Controller
{
  public:
  float gRes; 
  sensor_values sen_val;
  controller_values ctrl_val;

  float offset_x;
  float offset_y;

  bool init_once_;
  
  Controller();
  ~Controller();
  bool init(void);
  void readIMU(cIMU sensor, BallbotMotorDriver driver);
  float *computePsiDot(float omega_arr[]);
  float *computePsi(float psi_dot_arr[]);
  float *computePsi_new(float psi_real_arr[]);
  float *computePsi_new2(float psi_real_arr[]); 
  float *computePhiDot(float psi_dot_arr[]);
  float *computePhi(float psi_arr[], float theta_arr[]);
  float convert2radiand(float val_deg);
  float *executeController();
  float *executeController2();
  float *computeTorque(float curr_torque_arr[]);
  int16_t *compute2currentunits(float real_torques_arr[]);

  bool imu_init(cIMU sensor);
  
};

#endif


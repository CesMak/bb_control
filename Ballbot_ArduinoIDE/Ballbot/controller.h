#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Define.h>
#include <IMU.h>
#include <imu_spi.h>
#include <MadgwickAHRS.h>
#include <MPU9250.h>
#include <MPU9250_REGS.h>

#define VEL

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

  //y-Axis
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

  //z-Axis
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
  float K_yz_phi = -3.162; 
  float K_yz_theta = 24.8947; 
  float K_yz_phi_dot = -0.5533; 
  float K_yz_theta_dot = 7.1600;

  // K_values for y-direction
  float K_xz_phi = -3.162;
  float K_xz_theta = 24.8947;
  float K_xz_phi_dot = -0.5533;
  float K_xz_theta_dot = 7.1600;

  // controller values for z-direction
  float K_xy = 2.9800;
  float T_r = 1.0000;
  float T_n = 0.1000;

  // .....
  float diff = 0;
  float e = 0;
  float e_alt = 0;

  //Actuator Size
  float u =0;
};


void readIMU(cIMU device);
float convert2radiand(float val_deg);

//#class Controller
//#{
 // public:
  //#Controller();
  //#~Controller(); 
  //#sensor_values sen_val; 
  //#controller_values ctrl_val; 
  //#void readIMU(cIMU imu);
  //#float convert2radian(float x); 
  //#float getPhiValue(float theta, float psi);  
//#};

#endif


#include <math.h>
#include <Define.h>
#include <IMU.h>
#include <imu_spi.h>
#include <MadgwickAHRS.h>
#include <MPU9250.h>
#include <MPU9250_REGS.h>

#ifndef CONTROLLER_H
#define CONTROLLER_H



#define DEBUG

struct sensor_values{

  //x-Axis
  float phi_x_spoint = 0;
  float theta_x_spoint = 0;
  float psi_x_spoint = 0;

  float phi_x_cpoint = 0;
  float theta_x_cpoint = 0;
  float psi_x_cpoint = 0;

  //y-Axis
  float phi_y_spoint = 0;
  float theta_y_spoint = 0;
  float psi_y_spoint = 0;

  float phi_y_cpoint = 0;
  float theta_y_cpoint = 0;
  float psi_y_cpoint = 0;

  //z-Axis
  float phi_z_spoint = 0;
  float theta_z_spoint = 0;
  float psi_z_spoint = 0;

  float phi_z_cpoint = 0;
  float theta_z_cpoint = 0;
  float psi_z_cpoint = 0;
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



class Controller
{
  public:
  Controller();
  ~Controller(); 
  sensor_values sen_val; 
  controller_values ctrl_val; 
  void readIMU(cIMU imu);
  float convert2radian(float x); 
  float getPhiValue(float theta, float psi); 
  
};

#endif


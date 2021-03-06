#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Define.h>
#include <IMU.h> // /home/markus/.arduino15/packages/OpenCR/hardware/OpenCR/1.0.12/libraries/IMU
#include <imu_spi.h>
#include <MadgwickAHRS.h>
#include <MPU9250.h>
#include <MPU9250_REGS.h>

#include "ballbot_motor_driver.h"

#define SAMPL_TIME  10000      // in microseconds // if this value is low noise is increased! 
// Dead time is around 7ms!
#define A_PBZ       -2.660
#define K_EXP        222 // torque to unit factor 11.11 (gemessen - Michi) 222.2 (errechnet aus Datenblatt) 4.5 mNm/u, gemessen Markus: 4.3 mNM/u

#define ALPHA       PI/4            // je nach balldurchmesser unterschiedlich groß! siehe P. 33
#define BETA        -2*PI/3          // care this is correlated with the real wheel numbers! its teh angle from the x-axis of the IMU to the 1 real wheel
#define RK          0.07
#define RW          0.03

#define FAKT        1.8
#define X_OFFSET_RAD 0.05
#define Y_OFFSET_RAD -0.01

//#define DEBUG_SEN
#define PRINT_Values
//#define DEBUG_ANGLE
//#define DEBUG_VELOCITY
//#define DEBUG_PSI
//#define DEBUG_PSI_DOT

struct sensor_values {

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
  float T_r;
  float T_n;

  // .....
  float diff;
  float e;
  float e_alt;

  //Actuator Size
  float u;

  // real Torques:
  float T1;
  float T2;
  float T3;
};


class Controller
{
  public:
    float gRes;
    sensor_values sen_val;
    controller_values ctrl_val;

    float offset_x; // good values are: -2.09 y: 0.26
    float offset_y;

    bool init_once_;

    bool init_imu_filter_;
    float last_storage_x;
    float last_storage_y;
    float last_storage_z;
    float storage_x;
    float storage_y;
    float storage_z;
    float last_storage_x_dot;
    float last_storage_y_dot;
    float last_storage_z_dot;
    float storage_x_dot;
    float storage_y_dot;
    float storage_z_dot;

    bool torques_init_;
    float *last_torques_storage_;
    float *storage_torques_;

    Controller();
    ~Controller();
    void init(void);
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
    int *compute2currentunits(float real_torques_arr[]);

    bool imu_init(cIMU sensor, int samples);
    void xy_plane2D_controller(BallbotMotorDriver driver);
    void xyz_2D_controller(BallbotMotorDriver driver);
    void x_1D_controller(BallbotMotorDriver driver);
    void do_Step(BallbotMotorDriver driver);
    void test_IMU_FILTER(cIMU sensor);
    void imu_Filter(cIMU sensor, bool use_filter);
    float *torques_filter(float real_torques[], bool use_filter);

};

#endif


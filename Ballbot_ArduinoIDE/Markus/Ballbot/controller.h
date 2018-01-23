#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Define.h>
#include <IMU.h> // /home/markus/.arduino15/packages/OpenCR/hardware/OpenCR/1.0.12/libraries/IMU
#include <imu_spi.h>
#include <MadgwickAHRS.h>
#include <MPU9250.h>
#include <MPU9250_REGS.h>

#include "ballbot_motor_driver.h"


// Current Used Constants:
#define SAMPL_TIME  7000      // in microseconds // if this value is low noise is increased!  // Dead time is around 7ms! of the motors
#define FILTER_FAK  0.125     // ETHZ: 15 HZ rausfiltern das passt mit dem hier aber nicht überein! wenn wert auf 1 filter ist aus; für T=10ms & FilterFAK = 0.125 ca. 13HZ Cuttoff freq.
#define USE_FILTER  false         // =true -> use filtered value to apply torques  =false -> use unfiltered gyro and angle values to apply torques.
#define K_EXP       39        // torque to unit factor 11.11 (gemessen - Michi) 222.2 (errechnet aus Datenblatt) 4.5 mNm/u, gemessen Markus: 4.3 mNM/u
#define ALPHA       0.63879   // 36.6° je nach balldurchmesser unterschiedlich groß! siehe P. 33 markus gemessen für gelben ball zu 40° und mittelstellung der arme
#define BETA        -2*PI/3   // care this is correlated with the real wheel numbers! its teh angle from the x-axis of the IMU to the 1 real wheel
#define K1          -82.1341 // theta x
#define K2          -16.7913 // theta x dot
#define K3          -82.1341
#define K4          -16.7913
#define COS_ALPHA   cos(ALPHA)       // in rad.
#define SIN_ALPHA   sin(ALPHA)
#define SIN_BETA    -0.86602540378
#define COS_BETA    -0.5
#define SQRT3       1.73205080757
#define PRINT_TORQUES true  // if print as torques if false pritn as units!
#define USE_CUSTOM_OFFSET false  // use this offset!

#define RK          0.07
#define RW          0.03

//#define DEBUG_SEN
//#define DEBUG_ANGLE
//#define DEBUG_VELOCITY
//#define DEBUG_PSI
//#define DEBUG_PSI_DOT

struct sensor_values {

  float theta_x_cpoint;
  float theta_y_cpoint;
  float theta_z_cpoint;
  
  float theta_x_dot_cpoint;
  float theta_y_dot_cpoint;
  float theta_z_dot_cpoint;
  
  float theta_x_cpoint_ohne_Filter;
  float theta_y_cpoint_ohne_Filter;
  float theta_z_cpoint_ohne_Filter;

  float theta_y_dot_cpoint_ohne_Filter;
  float theta_x_dot_cpoint_ohne_Filter;
  float theta_z_dot_cpoint_ohne_Filter;
};


struct controller_values
{
  // real Torques:
  float T1;
  float T2;
  float T3;

  float T1_ohne_Filter;
  float T2_ohne_Filter;
  float T3_ohne_Filter;
};


class Controller
{
  public:
    float gRes;
    sensor_values sen_val;
    controller_values ctrl_val;

    float offset_x; // good values are: -2.09 y: 0.26
    float offset_y;
    float custom_offset_x;
    float custom_offset_y;

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
    void imu_Filter(cIMU sensor);
    float *torques_filter(float real_torques[], bool use_filter);

};

#endif


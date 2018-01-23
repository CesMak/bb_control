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
#define USE_FILTER  false     // =true -> use filtered value to apply torques  =false -> use unfiltered gyro and angle values to apply torques.
#define K_EXP       39        // torque to unit factor 11.11 (gemessen - Michi) 222.2 (errechnet aus Datenblatt) 4.5 mNm/u, gemessen Markus: 4.3 mNM/u
#define ALPHA       PI/4      // 45° ist nur von Konstruktion Abhängig sollte auf Ballgröße angepasst werden.
#define BETA        PI        // care this is correlated with the real wheel numbers! its teh angle from the x-axis of the IMU to the 1 real wheel
#define K1          0         // tune phi_x, phi_y, set to zero if you do not wanna use the ball's odometry  -0.3162
#define K2          10        // tune theta_x, theta_y, set to zero if you do not wanna use the ball's odometry
#define K3          0         // tune dphi_x, dphi_y, set to zero if you do not wanna use the ball's odometry  -0.3991
#define K4          3         // tune dtheta_x, dtheta_y, set to zero if you do not wanna use the ball's odometry
#define COS_ALPHA   cos(ALPHA)       // in rad.
#define SIN_ALPHA   sin(ALPHA)
#define SIN_BETA    sin(BETA)
#define COS_BETA    cos(BETA)
#define SQRT3       1.73205080757
#define PRINT_TORQUES true  // if print as torques if false pritn as units!
#define USE_CUSTOM_OFFSET false  // use this offset!
#define RK         0.08    // radius of ball in meter
#define RW         0.03    // radius of omni-wheel in meter

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

  float theta_x_dot_cpoint_ohne_Filter;
  float theta_y_dot_cpoint_ohne_Filter;
  float theta_z_dot_cpoint_ohne_Filter;

  // Odometry: 
  float *psi_actual_;
  float *psi_last_;

  float *dphi_;
  float *phi_actual_;
  float *phi_last_;
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
    void calc_odometry(int32_t velocity_RAW[]);

};

#endif


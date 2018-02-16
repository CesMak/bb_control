
#include "controller.h"

extern BallbotMotorDriver motor_driver;
extern cIMU imu;


Controller::Controller()
{

}

Controller::~Controller()
{

}


void Controller::init(void)
{
  sen_val = {0, 0, 0, 0, 0, 0};

  ctrl_val.T1             = 0.0;
  ctrl_val.T2             = 0.0;
  ctrl_val.T3             = 0.0;

  ctrl_val.T1_ohne_Filter             = 0.0;
  ctrl_val.T2_ohne_Filter             = 0.0;
  ctrl_val.T3_ohne_Filter             = 0.0;

  //Factor to convert gyroscope data to angular vel.
  gRes = 2000.0 / 32768.0;

  // for filter:
  last_torques_storage_ = new float[3];
  storage_torques_ = new float[3];

  custom_offset_x = 0.6; // in grad!
  custom_offset_x = -2;

  sen_val.psi_actual_ = new float[3];
  sen_val.psi_last_   = new float[3];
  sen_val.phi_actual_ = new float[3];
  sen_val.phi_last_   = new float[3];
  sen_val.dphi_       = new float[3];

  for (int i = 0; i < 3; i++) {
    sen_val.psi_actual_[i] = 0.0;
    sen_val.psi_last_[i]   = 0.0;
    sen_val.phi_actual_[i] = 0.0;
    sen_val.phi_last_[i]   = 0.0;
    sen_val.dphi_[i]       = 0.0;
  }
}

void Controller::read_values(cIMU sensor)
{
  //Theta y,x,z in radiands
  sen_val.theta_x_cpoint = convert2radiand(sensor.rpy[1]);
  sen_val.theta_y_cpoint = convert2radiand(sensor.rpy[0]);
  sen_val.theta_z_cpoint = convert2radiand(sensor.rpy[2]);

  // Theta_dot y,x,z
  // TODO gRES FAKTOR?!
  sen_val.theta_x_dot_cpoint = convert2radiand(sensor.gyroData[1] * gRes);
  sen_val.theta_y_dot_cpoint = convert2radiand(sensor.gyroData[0] * gRes);
  sen_val.theta_z_dot_cpoint = convert2radiand(sensor.gyroData[2] * gRes);
  //driver.readWheelStates(current_effort_RAW, current_velocity_RAW, current_position_RAW); // this step costs 3ms!!!
}



void Controller::readIMU(cIMU sensor, BallbotMotorDriver driver)
{
  float time_start = millis();

    read_values(sensor);
    
    // Execute Controller
    calc_values(driver); // lasts 3ms the write wheel states lasts that long!
}

void Controller::calc_values(BallbotMotorDriver driver)
{
  //Compute Tx,Ty
  static float* virtual_torques = new float[2];
  static float* real_torques = new float[2];

  // Torque in the yz Planar --> T_x
  virtual_torques[0] = (sen_val.phi_actual_[0] * K1_X + sen_val.theta_x_cpoint * K2_X + sen_val.dphi_[0] * K3_X + sen_val.theta_x_dot_cpoint * K4_X ) * -1;

  // Torque in the xz Planar --> T_y
  virtual_torques[1] = (sen_val.phi_actual_[1] * K1_Y + sen_val.theta_y_cpoint * K2_Y + sen_val.dphi_[1] * K3_Y + sen_val.theta_y_dot_cpoint * K4_Y ) * -1;

  virtual_torques[2] = 0.0; //(sen_val.theta_z_cpoint *  1.0 + sen_val.theta_z_dot_cpoint * 1.7055) * -1;

  real_torques[0] = 0.333333333 * (virtual_torques[2] + (2 / COS_ALPHA) * (virtual_torques[0] * COS_BETA - virtual_torques[1] * SIN_BETA));
  real_torques[1] = 0.333333333 * (virtual_torques[2] + (1 / COS_ALPHA) * (SIN_BETA * (-virtual_torques[0] * SQRT3 + virtual_torques[1]) - COS_BETA * (virtual_torques[0] + SQRT3 * virtual_torques[1])));
  real_torques[2] = 0.333333333 * (virtual_torques[2] + (1 / COS_ALPHA) * (SIN_BETA * (virtual_torques[0] * SQRT3 + virtual_torques[1]) + COS_BETA * (-virtual_torques[0] + SQRT3 * virtual_torques[1])));

  //Convert real Torques into Current
  int* curr_unit_arr = compute2currentunits(real_torques);

  // Write Values to motors:
  driver.writeServoConfig(DXM_1_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[0]);
  driver.writeServoConfig(DXM_2_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[1]);
  driver.writeServoConfig(DXM_3_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[2]);
}

float Controller::convert2radiand(float val_deg)
{
  return (val_deg * PI) / 180;
}

int *Controller::compute2currentunits(float real_torques_arr[])
{

  static int* ret_arr = new int[3];

  ret_arr[0] = round(K_M1 * real_torques_arr[0]);
  ret_arr[1] = round(K_M2 * real_torques_arr[1]);
  ret_arr[2] = round(K_M3 * real_torques_arr[2]);
  
  return ret_arr;
}

void Controller::calc_odometry(int32_t velocity_RAW[])
{
  // 0. Convert the velocity from units in rad per second
  // 1 Unit = 0.229[RPM]
  float velocity_rad_s[] = {0.0, 0.0, 0.0};
  float dpsi[] = {0.0, 0.0, 0.0};
  for (int i = 0; i < 3; i++)
  {
    velocity_rad_s[i] = velocity_RAW[i] * 0.229 * (2 * PI) / 60;
  }
  Serial.print("velocity_rad_s_0:  "); Serial.println(velocity_rad_s[0]);

  // 1. Convert dpsi_1,2,3 -> dpsi_x,y,z
  // 1.A dpsix = dpsi1 / cos (alpha) P.71
  // 1.B dpsiy = dpsi3 * 2 / (sqrt(3) *cos(alpha))
  // 1.C dpsiz = dpsi1/sin(alpha)
  // diese Formeln sind nicht richtig! hängt noch von Beta ab und Formeln müssen evtl. summiert werden.
  //
  dpsi[0] = velocity_rad_s[0] / (cos(ALPHA));
  dpsi[1] = velocity_rad_s[2] / (cos(ALPHA) * SQRT3);
  dpsi[2] = velocity_rad_s[1] / (sin(ALPHA));

  Serial.print("dpsi_0:  "); Serial.println(dpsi[0]);

  // 2. Use dpsi_x,y,z -> psi_x,y,z (INTEGRATE)
  // psi_k = psi_k-1 + dpsi * sample_time
  // Integration like ETHZ: see: https://en.wikipedia.org/wiki/Trapezoidal_rule
  sen_val.psi_actual_[0] = sen_val.psi_last_[0] + SAMPL_TIME * pow(10, -6) * dpsi[0];
  sen_val.psi_actual_[1] = sen_val.psi_last_[1] + SAMPL_TIME * pow(10, -6) * dpsi[1];
  sen_val.psi_actual_[2] = sen_val.psi_last_[2] + SAMPL_TIME * pow(10, -6) * dpsi[2];

  Serial.print("sen_val.psi_actual_[0]:  "); Serial.println(sen_val.psi_actual_[0]);
  Serial.print("sen_val.psi_last_[0]:  "); Serial.println(sen_val.psi_last_[0]);

  // 3. psi_x,y,z -> dphi_x,y,z P. 8
  sen_val.dphi_[0] = (dpsi[0] + sen_val.theta_x_dot_cpoint_ohne_Filter) * RW / RK + sen_val.theta_x_dot_cpoint_ohne_Filter;
  sen_val.dphi_[1] = (dpsi[1] + sen_val.theta_y_dot_cpoint_ohne_Filter) * RW / RK + sen_val.theta_y_dot_cpoint_ohne_Filter;
  sen_val.dphi_[2] = (dpsi[2] / sin(ALPHA)) * RW / RK + sen_val.theta_z_dot_cpoint_ohne_Filter;

  // 4. dphi_x,y,z -> phi_x,y,z (integrate it!)
  sen_val.phi_actual_[0] = sen_val.phi_last_[0] + SAMPL_TIME * pow(10, -6) * sen_val.dphi_[0];
  sen_val.phi_actual_[1] = sen_val.phi_last_[0] + SAMPL_TIME * pow(10, -6) * sen_val.dphi_[1];
  sen_val.phi_actual_[2] = sen_val.phi_last_[0] + SAMPL_TIME * pow(10, -6) * sen_val.dphi_[2];

  Serial.print("sen_val.dphi_[0]:  "); Serial.println(sen_val.dphi_[0]);
  Serial.print("sen_val.phi_actual_[0]:  "); Serial.println(sen_val.phi_actual_[0]);
  Serial.print("sen_val.phi_last_[0]:  ");   Serial.println(sen_val.phi_last_[0]);


  sen_val.psi_last_ = sen_val.psi_actual_;
  sen_val.phi_last_ = sen_val.phi_actual_;
}


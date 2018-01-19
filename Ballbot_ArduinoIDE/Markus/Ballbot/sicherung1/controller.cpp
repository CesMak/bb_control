
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
  sen_val = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double theta = 24;
  double phi = 7;

  // K_values for x-direction
  ctrl_val.K_yz_phi       = -3.162;
  ctrl_val.K_yz_theta     = theta * FAKT;
  ctrl_val.K_yz_phi_dot   = -0.5533;
  ctrl_val.K_yz_theta_dot = phi * FAKT;

  // K_values for y-direction
  ctrl_val.K_xz_phi       = -3.162;
  ctrl_val.K_xz_theta     = theta * FAKT;
  ctrl_val.K_xz_phi_dot   = -0.5533;
  ctrl_val.K_xz_theta_dot = phi * FAKT;

  // controller values for z-direction
  ctrl_val.K_xy          = 0;
  ctrl_val.T_r           = 0;
  ctrl_val.T_n           = 0;

  // .....
  ctrl_val.diff          = 0;
  ctrl_val.e             = 0;
  ctrl_val.e_alt         = 0;

  ctrl_val.u             = 0;

  ctrl_val.T1             = 0.0;
  ctrl_val.T2             = 0.0;
  ctrl_val.T3             = 0.0;

  //Factor to convert gyroscope data to angular vel.
  gRes = 2000.0 / 32768.0;

  last_torques_storage_ = new float[3];
  storage_torques_ = new float[3];
}

bool Controller::imu_init(cIMU sensor, int samples)
{
  // with samples you can adjust how long the init process will last.
  float storage_imux = 0.0;
  float storage_imuy = 0.0;


  static uint32_t tTime = 0;
  int counter = 0;


  Serial.println("Start IMU init. (Wait some time!) ");

  while (counter < samples) {

    if (millis() - tTime > 60 && abs(sensor.rpy[0]) < 4 && abs(sensor.rpy[1]) < 4 ) {
      tTime = millis();
      sensor.update();
      storage_imux += sensor.rpy[1];
      storage_imuy += sensor.rpy[0];
      counter++;
      if (counter == samples / 4)  {
        Serial.print("IMU INIT 25% finished with current offset: ");
        Serial.print(storage_imux / (0.25 * samples));
        Serial.print(" ");
        Serial.println(storage_imuy / (0.25 * samples));
      }
      if (counter == samples / 2)  {
        Serial.print("IMU INIT 50% finished with current offset: ");
        Serial.print(storage_imux / (0.5 * samples));
        Serial.print(" ");
        Serial.println(storage_imuy / (0.5 * samples));
      }
      if (counter == 3 * samples / 4) {
        Serial.print("IMU INIT 75% finished with current offset: ");
        Serial.print(storage_imux / (0.75 * samples));
        Serial.print(" ");
        Serial.println(storage_imuy / (0.75 * samples));
      }
    }
  }
  offset_x = storage_imux / samples;
  offset_y = storage_imuy / samples;

  Serial.print("IMU INIT 100% finished with offset: "); Serial.print(offset_x); Serial.print(" y: "); Serial.println(offset_y);
  return true;
}


void Controller::imu_Filter(cIMU sensor, bool use_filter)
{
  // 1. Init step do this only once:
  if (use_filter)
  {
    if (!init_imu_filter_)
    {
      sensor.update();
      float last_storage_x = sensor.rpy[1];
      float last_storage_y = sensor.rpy[0];
      float last_storage_z = sensor.rpy[2];

      float storage_x = sensor.rpy[1];
      float storage_y = sensor.rpy[0];
      float storage_z = sensor.rpy[2];

      float last_storage_x_dot = sensor.gyroData[1];
      float last_storage_y_dot = sensor.gyroData[0];
      float last_storage_z_dot = sensor.gyroData[2];

      float storage_x_dot = sensor.gyroData[1];
      float storage_y_dot = sensor.gyroData[0];
      float storage_z_dot = sensor.gyroData[2];
      init_imu_filter_ = true;

      //Theta y,x,z in radiands
      sen_val.theta_x_cpoint = convert2radiand(storage_x - offset_x);
      sen_val.theta_y_cpoint = convert2radiand(storage_y - offset_y);
      sen_val.theta_z_cpoint = convert2radiand(storage_z);

      // Theta_dot y,x,z
      sen_val.theta_x_dot_cpoint = convert2radiand(storage_x_dot * gRes);
      sen_val.theta_y_dot_cpoint = convert2radiand(storage_y_dot * gRes);
      sen_val.theta_z_dot_cpoint = convert2radiand(storage_z_dot * gRes);
      return;//delay(8);
    }
    float filtervalue = 0.01; // entspricht  0.0666 15Hz wie ETHZ
    sensor.update();
    storage_x = last_storage_x - ( last_storage_x - sensor.rpy[1] ) * filtervalue;
    storage_y = last_storage_y - ( last_storage_y - sensor.rpy[0] ) * filtervalue;
    storage_z = last_storage_z - ( last_storage_z - sensor.rpy[2] ) * filtervalue;
    last_storage_x = storage_x;
    last_storage_y = storage_y;
    last_storage_z = storage_z;

    storage_x_dot = last_storage_x_dot - ( last_storage_x_dot - sensor.gyroData[1] ) * filtervalue;
    storage_y_dot = last_storage_y_dot - ( last_storage_y_dot - sensor.gyroData[0] ) * filtervalue;
    storage_z_dot = last_storage_z_dot - ( last_storage_z_dot - sensor.gyroData[2] ) * filtervalue;
    last_storage_x_dot = storage_x_dot;
    last_storage_y_dot = storage_y_dot;
    last_storage_z_dot = storage_z_dot;

    //Theta y,x,z in radiands
    sen_val.theta_x_cpoint = convert2radiand(storage_x - offset_x);
    sen_val.theta_y_cpoint = convert2radiand(storage_y - offset_y);
    sen_val.theta_z_cpoint = convert2radiand(storage_z);

    // Theta_dot y,x,z
    sen_val.theta_x_dot_cpoint = convert2radiand(storage_x_dot * gRes);
    sen_val.theta_y_dot_cpoint = convert2radiand(storage_y_dot * gRes);
    sen_val.theta_z_dot_cpoint = convert2radiand(storage_z_dot * gRes);
  }

  else
  {
    //Theta y,x,z in radiands
    sen_val.theta_x_cpoint = convert2radiand(sensor.rpy[1] - offset_x);
    sen_val.theta_y_cpoint = convert2radiand(sensor.rpy[0] - offset_y);
    sen_val.theta_z_cpoint = convert2radiand(sensor.rpy[2]);

    // Theta_dot y,x,z
    sen_val.theta_y_dot_cpoint = convert2radiand(sensor.gyroData[0] * gRes);
    sen_val.theta_x_dot_cpoint = convert2radiand(sensor.gyroData[1] * gRes);
    sen_val.theta_z_dot_cpoint = convert2radiand(sensor.gyroData[2] * gRes);
  }

}



void Controller::readIMU(cIMU sensor, BallbotMotorDriver driver)
{
  float time_start = millis();

  //dass motor nicht gleich losfährt wegen anfangs IMU offset.
  if ( (abs(sensor.rpy[1] - (offset_x)) < 0.3 && abs(sensor.rpy[0] - (offset_y)) < 0.3) || init_once_)
  {
    imu_Filter(sensor, true);
    init_once_ = true;

    // Read out wheel states (current[units], velocity, position)
    // CAUTION at least the effort value is only given positive or if negative as 65526
    int32_t current_effort_RAW[3]   =  {0.0, 0.0, 0.0}; // this is the present current
    int32_t current_velocity_RAW[3] =  {0.0, 0.0, 0.0};
    int32_t current_position_RAW[3] =  {0.0, 0.0, 0.0};

    driver.readWheelStates(current_effort_RAW, current_velocity_RAW, current_position_RAW);

    // convert negative values of current_effort_RAW:
    if (current_effort_RAW[0] > 10000)
    {
      current_effort_RAW[0] = current_effort_RAW[0] - 65536;
    }

    if (current_effort_RAW[1] > 10000)
    {
      current_effort_RAW[1] = current_effort_RAW[1] - 65536;
    }

    if (current_effort_RAW[2] > 10000)
    {
      current_effort_RAW[2] = current_effort_RAW[2] - 65536;
    }

    // Execute Controller
    // this contains a filter!
    xyz_2D_controller(driver);

    float time_end = millis();

    float time_duration = time_end - time_start;

#ifdef PRINT_Values
    Serial.print(time_end / 1000);                                Serial.print("\t"); // sec
    Serial.print(sen_val.theta_x_cpoint * 180 / 3.14159);         Serial.print("\t"); // °
    Serial.print(sen_val.theta_x_dot_cpoint * 180 / 3.14159);     Serial.print("\t"); // °/sec
    Serial.print(sen_val.theta_y_cpoint * 180 / 3.14159);         Serial.print("\t");
    Serial.print(sen_val.theta_y_dot_cpoint * 180 / 3.14159);     Serial.print("\t");

    Serial.print(current_effort_RAW[0]);                         Serial.print("\t"); // effort in units gemessen!
    Serial.print(ctrl_val.T1 * K_EXP);                           Serial.print("\t"); // effort in units drauf
    Serial.print(current_effort_RAW[1]);                         Serial.print("\t"); // effort in units gemessen!
    Serial.print(ctrl_val.T2 * K_EXP);                           Serial.print("\t"); // effort in units drauf
    Serial.print(current_effort_RAW[2]);                         Serial.print("\t"); // effort in units gemessen!
    Serial.print(ctrl_val.T3 * K_EXP);                           Serial.print("\t"); // effort in units drauf

    Serial.print(time_duration);
    //
    //curr_unit_arr
    Serial.print("\n");
#endif
  }
}


void Controller::do_Step(BallbotMotorDriver driver)
{
  // dead time is around 7ms.
  int32_t current_effort_RAW[3]   =  {0.0,   0.0, 0.0}; // this is the present current
  int32_t current_velocity_RAW[3] =  {0.0, 0.0, 0.0};
  int32_t current_position_RAW[3] =  {0.0, 0.0, 0.0};
  driver.writeServoConfig(DXM_1_ID, 2 , ADDR_X_GOAL_EFFORT , 100);
  while (1) {
    driver.readWheelStates(current_effort_RAW, current_velocity_RAW, current_position_RAW);
    Serial.print(current_effort_RAW[0]); Serial.print("\t");
    Serial.println(millis());
  }
}

void Controller::x_1D_controller(BallbotMotorDriver driver)
{
  //Compute Tx,Ty
  static float* virtual_torques = new float[2];

  //Torque in the yz Planar --> T_x
  virtual_torques[0] = (sen_val.theta_x_cpoint * 14.6965 +  sen_val.theta_x_dot_cpoint * 3.6623) * -1;

  // Torque in the xz Planar --> T_y
  virtual_torques[1] = 0.0;//(sen_val.theta_y_cpoint *  14.6965 + sen_val.theta_y_dot_cpoint * 3.6623) * -1;

  virtual_torques[2] = 0.0; //(sen_val.theta_z_cpoint *  1.0 + sen_val.theta_z_dot_cpoint * 1.7055) * -1;

  static float* real_torques = new float[2];

  real_torques[0] = 0.3333333333333 * (virtual_torques[2] + (2 / cos(ALPHA)) * (virtual_torques[0] * cos(BETA) - virtual_torques[1] * sin(BETA)));

  //compute Torque T2 P14.
  real_torques[1] = 0.3333333333333 * (virtual_torques[2] + (1 / cos(ALPHA)) * (sin(BETA) * (-virtual_torques[0] * sqrt(3) + virtual_torques[1]) - cos(BETA) * (virtual_torques[0] + sqrt(3) * virtual_torques[1])));

  //compute Torque T3
  real_torques[2] = 0.3333333333333 * (virtual_torques[2] + (1 / cos(ALPHA)) * (sin(BETA) * (virtual_torques[0] * sqrt(3) + virtual_torques[1]) + cos(BETA) * (-virtual_torques[0] + sqrt(3) * virtual_torques[1])));


  // filter real torques right here

  //Convert real Torques into Current
  int* curr_unit_arr = compute2currentunits(real_torques);

  //Load to motors
  driver.writeServoConfig(DXM_1_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[0]);
  driver.writeServoConfig(DXM_2_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[1]);
  driver.writeServoConfig(DXM_3_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[2]);

  ctrl_val.T1 = real_torques[0];
  ctrl_val.T2 = real_torques[1];
  ctrl_val.T3 = real_torques[2];
}

void Controller::xyz_2D_controller(BallbotMotorDriver driver)
{
  //Compute Tx,Ty
  static float* virtual_torques = new float[2];

  //Torque in the yz Planar --> T_x
  virtual_torques[0] = (sen_val.theta_x_cpoint * 39 +  sen_val.theta_x_dot_cpoint * 11) * -1;

  // Torque in the xz Planar --> T_y
  virtual_torques[1] = (sen_val.theta_y_cpoint * 39 + sen_val.theta_y_dot_cpoint * 11) * -1;

  virtual_torques[2] = 0.0; //(sen_val.theta_z_cpoint *  1.0 + sen_val.theta_z_dot_cpoint * 1.7055) * -1;

  static float* real_torques = new float[2];

  real_torques[0] = 0.333333333 * (virtual_torques[2] + (2 / cos(ALPHA)) * (virtual_torques[0] * cos(BETA) - virtual_torques[1] * sin(BETA)));

  //compute Torque T2
  real_torques[1] = 0.333333333 * (virtual_torques[2] + (1 / cos(ALPHA)) * (sin(BETA) * (-virtual_torques[0] * 

 + virtual_torques[1]) - cos(BETA) * (virtual_torques[0] + sqrt(3) * virtual_torques[1])));

  //compute Torque T3
  real_torques[2] = 0.333333333 * (virtual_torques[2] + (1 / cos(ALPHA)) * (sin(BETA) * (virtual_torques[0] * sqrt(3) + virtual_torques[1]) + cos(BETA) * (-virtual_torques[0] + sqrt(3) * virtual_torques[1])));

  // filter real torques right here
  //real_torques = torques_filter(real_torques, true);

  //Convert real Torques into Current
  int* curr_unit_arr = compute2currentunits(real_torques);

  //Load to motors
  driver.writeServoConfig(DXM_1_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[0]);
  driver.writeServoConfig(DXM_2_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[1]);
  driver.writeServoConfig(DXM_3_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[2]);

  ctrl_val.T1 = real_torques[0];
  ctrl_val.T2 = real_torques[1];
  ctrl_val.T3 = real_torques[2];
}

float* Controller::torques_filter(float real_torques[], bool use_filter)
{
  if (use_filter)
  {
    if (!torques_init_)
    {
      last_torques_storage_ = real_torques;
      storage_torques_ = real_torques;
      torques_init_ = true;
      return real_torques;
    }
    for (int i = 0; i < 3; i++)
    {
      storage_torques_[i] = last_torques_storage_[i] - ( last_torques_storage_[i] - real_torques[i]) * 0.1;
      last_torques_storage_[i] = storage_torques_[i];
    }
  }
  else
  {
    return real_torques;
  }
  return storage_torques_;
}

void Controller::xy_plane2D_controller(BallbotMotorDriver driver)
{
  //Compute Tx,Ty
  static float* virtual_torques = new float[2];

  //Torque in the yz Planar --> T_x
  virtual_torques[0] = (sen_val.theta_x_cpoint * ctrl_val.K_yz_theta +  sen_val.theta_x_dot_cpoint * ctrl_val.K_yz_theta_dot) * -1;

  // Torque in the xz Planar --> T_y
  virtual_torques[1] = (sen_val.theta_y_cpoint * ctrl_val.K_xz_theta + sen_val.theta_y_dot_cpoint * ctrl_val.K_xz_theta_dot) * -1;

  virtual_torques[2] = 0.0;

  static float* real_torques = new float[2];

  real_torques[0] = 0.333333333 * (virtual_torques[2] + (2 / cos(ALPHA)) * (virtual_torques[0] * cos(BETA) - virtual_torques[1] * sin(BETA)));

  //compute Torque T2
  real_torques[1] = 0.333333333 * (virtual_torques[2] + (1 / cos(ALPHA)) * (sin(BETA) * (-virtual_torques[0] * sqrt(3) + virtual_torques[1]) - cos(BETA) * (virtual_torques[0] + sqrt(3) * virtual_torques[1])));

  //compute Torque T3
  real_torques[2] = 0.333333333 * (virtual_torques[2] + (1 / cos(ALPHA)) * (sin(BETA) * (virtual_torques[0] * sqrt(3) + virtual_torques[1]) + cos(BETA) * (-virtual_torques[0] + sqrt(3) * virtual_torques[1])));

  //Convert real Torques into Current
  int* curr_unit_arr = compute2currentunits(real_torques);

  //Load to motors
  driver.writeServoConfig(DXM_1_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[0]);
  driver.writeServoConfig(DXM_2_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[1]);
  driver.writeServoConfig(DXM_3_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[2]);

  ctrl_val.T1 = real_torques[0];
  ctrl_val.T2 = real_torques[1];
  ctrl_val.T3 = real_torques[2];
}

void Controller::test_IMU_FILTER(cIMU sensor)
{
  // 1. Init step:
  sensor.update();
  float last_storage_x = sensor.rpy[1];
  float storage_x = sensor.rpy[1];

  float last_storage_y = sensor.rpy[0];
  float storage_y = sensor.rpy[0];
  float last_storage_z = sensor.rpy[2];
  float storage_z = sensor.rpy[2];
  while (1)
  {
    delay(8);
    sensor.update();
    storage_x = last_storage_x - ( last_storage_x - sensor.rpy[1] ) * 0.125;
    storage_y = last_storage_y - ( last_storage_y - sensor.rpy[0] ) * 0.125;
    storage_z = last_storage_z - ( last_storage_z - sensor.rpy[2] ) * 0.125;
    last_storage_x = storage_x;
    last_storage_y = storage_y;
    last_storage_z = storage_z;
    Serial.print(millis()); Serial.print("\t"); Serial.print(sensor.rpy[1]); Serial.print("\t"); Serial.println(storage_x);
  }
}

float  *Controller::computePsiDot(float omega_arr[])
{
  float *ret_arr = new float[3];

  //Psi_dot_x
  ret_arr[0] = omega_arr[0] / cos(ALPHA);

  //Psi_dot_x
  ret_arr[1] = (omega_arr[2] * 2) / (sqrt(3) * cos(ALPHA));

  //Psi_dot_z
  ret_arr[2] = (omega_arr[0]) / (sin(ALPHA));


  return ret_arr;
}

float *Controller::computePsi(float psi_dot_arr[])
{
  float *ret_arr = new float[3];
  static float values_psi_x[2] = {0, 0};
  static float values_psi_y[2] = {0, 0};
  static float values_psi_z[2] = {0, 0};

  //Aktuelle Werte zuweisen
  static float current_value_psi_x_dot = psi_dot_arr[0];
  static float current_value_psi_y_dot = psi_dot_arr[1];
  static float current_value_psi_z_dot = psi_dot_arr[2];

  // Psi_x
  values_psi_x[0] = current_value_psi_x_dot * SAMPL_TIME + values_psi_x[1];
  values_psi_x[1] = values_psi_x[0];
  ret_arr[0] = values_psi_x[0];

  // Psi_y
  values_psi_y[0] = current_value_psi_y_dot * SAMPL_TIME + values_psi_y[1];
  values_psi_y[1] = values_psi_y[0];
  ret_arr[1] = values_psi_y[0];

  // Psi_z
  values_psi_z[0] = current_value_psi_z_dot * SAMPL_TIME + values_psi_z[1];
  values_psi_z[1] = values_psi_z[0];
  ret_arr[2] = values_psi_z[0];


  return ret_arr;


}

float *Controller::computePsi_new2(float psi_real_arr[]) {

  float *ret_arr = new float[3];

  ret_arr[0] = psi_real_arr[0] / cos(ALPHA);
  ret_arr[1] = (psi_real_arr[2] * 2) / (sqrt(3) * cos(ALPHA));
  ret_arr[2] = psi_real_arr[0] / sin(ALPHA);

  return ret_arr;
}

float *Controller::computePsi_new(float psi_real_arr[]) {

  float *ret_arr = new float[3];

  static float values_psi_1[2] = {0, 0};
  static float values_psi_2[2] = {0, 0};
  static float values_psi_3[2] = {0, 0};

  static float values_psi_x[2] = {0, 0};
  static float values_psi_y[2] = {0, 0};
  static float values_psi_z[2] = {0, 0};

  //aktuellen Werte einlesen
  values_psi_1[0] = psi_real_arr[0];
  values_psi_1[1] = psi_real_arr[1];
  values_psi_1[2] = psi_real_arr[2];

  values_psi_x[0] = values_psi_x[1] + ((values_psi_1[0] - values_psi_1[1]) / cos(ALPHA));

  values_psi_x[1] = values_psi_x[0];
  values_psi_1[1] = values_psi_1[0];
  ret_arr[0] = values_psi_x[0];

  values_psi_y[0] = values_psi_y[1] + ((values_psi_3[0] - values_psi_3[1]) * 2 / (sqrt(3) * cos(ALPHA)));

  values_psi_y[1] = values_psi_y[0];
  values_psi_3[1] = values_psi_3[0];
  ret_arr[1] = values_psi_y[0];


  values_psi_z[0] = values_psi_z[1] + ((values_psi_2[0] - values_psi_3[1]) / sin(ALPHA));
  values_psi_z[1] = values_psi_z[0];
  values_psi_2[1] = values_psi_2[0];
  ret_arr[2] = values_psi_z[0];

  return ret_arr;

}

float *Controller::computePhiDot(float psi_dot_arr[])
{
  float *ret_arr = new float[3];

  //Phi_dot_x
  ret_arr[0] = sen_val.theta_x_dot_cpoint + ((psi_dot_arr[0] + sen_val.theta_x_dot_cpoint) * (RW / RK));

  //Phi_dot_y
  ret_arr[1] = sen_val.theta_y_dot_cpoint + ((psi_dot_arr[1] + sen_val.theta_y_dot_cpoint) * (RW / RK));

  //Phi_dot_z
  ret_arr[2] = sen_val.theta_z_dot_cpoint * ((RW) / (RK * sin(ALPHA))) + sen_val.theta_z_dot_cpoint;

  return ret_arr;
}

float *Controller::computePhi(float psi_arr[], float theta_arr[])
{
  float *ret_arr = new float[3];

  static float values_psi_x[2] = {0, 0};
  static float values_psi_y[2] = {0, 0};
  static float values_psi_z[2] = {0, 0};

  static float values_theta_x[2] = {0, 0};
  static float values_theta_y[2] = {0, 0};
  static float values_theta_z[2] = {0, 0};

  static float values_phi_x[2] = {0, 0};
  static float values_phi_y[2] = {0, 0};
  static float values_phi_z[2] = {0, 0};

  //aktuelle Werte Psi_x_Werte zuweisen
  values_psi_x[0] = psi_arr[0];
  values_psi_y[0] = psi_arr[1];
  values_psi_z[0] = psi_arr[2];

  values_theta_x[0] = theta_arr[0];
  values_theta_y[0] = theta_arr[1];
  values_theta_z[0] = theta_arr[2];

  //Computation of Phi_x
  values_phi_x[0] = (values_psi_x[0] - values_psi_x[1] - values_theta_x[1] + values_theta_x[0]) * (RW / RK) + values_theta_x[0] - values_theta_x[1] + values_phi_x[1];
  values_phi_y[0] = (values_psi_y[0] - values_psi_y[1] + values_theta_y[1] - values_theta_y[0]) * (RW / RK) + values_theta_y[0] - values_theta_y[1] + values_phi_y[1];
  values_phi_z[0] = (values_psi_x[0] - values_psi_x[1]) * ((RW) / (RK * sin(ALPHA))) + values_theta_x[0] - values_theta_x[1] + values_phi_x[1];

  //Switching values

  values_psi_x[1] = values_psi_x[0];
  values_theta_x[1] = values_theta_x[0];
  values_phi_x[1] = values_phi_x[0];
  ret_arr[0] = values_phi_x[0];

  values_psi_y[1] = values_psi_y[0];
  values_theta_y[1] = values_theta_y[0];
  values_phi_y[1] = values_phi_y[0];
  ret_arr[1] = values_phi_y[0];

  values_psi_z[1] = values_psi_z[0];
  values_theta_z[1] = values_theta_z[0];
  values_phi_z[1] = values_phi_z[0];
  ret_arr[2] = values_phi_z[0];

  return ret_arr;
}


float Controller::convert2radiand(float val_deg)
{
  float temp_var = (val_deg * 2 * PI) / 360;
  return temp_var;
}

float *Controller::executeController()
{
  static float* ret_arr = new float[3];

  //Torque in the yz Planar --> T_x
  ret_arr[0] = (sen_val.phi_x_cpoint * ctrl_val.K_yz_phi + sen_val.theta_x_cpoint * ctrl_val.K_yz_theta + sen_val.phi_x_dot_cpoint * ctrl_val.K_yz_phi_dot + sen_val.theta_x_dot_cpoint * ctrl_val.K_yz_theta_dot) * -1;

  // Torque in the xz Planar --> T_y
  ret_arr[1] = (sen_val.phi_y_cpoint * ctrl_val.K_xz_phi + sen_val.theta_y_cpoint * ctrl_val.K_xz_theta + sen_val.phi_y_dot_cpoint * ctrl_val.K_xz_phi_dot + sen_val.theta_y_dot_cpoint * ctrl_val.K_xz_theta_dot) * -1;

  // Torque in the xz Planar --> T_z
  static float e[2] = {0, 0};

  //current value of difference e
  e[0] = sen_val.theta_z_cpoint;

  ret_arr[2] = A_PBZ * e[1];

  //shift values
  e[1] = e[0];

  return ret_arr;
}

float *Controller::executeController2()
{
  static float* ret_arr = new float[2];

  //Torque in the yz Planar --> T_x
  ret_arr[0] = (sen_val.theta_x_cpoint * ctrl_val.K_yz_theta +  sen_val.theta_x_dot_cpoint * ctrl_val.K_yz_theta_dot) * -1;

  // Torque in the xz Planar --> T_y
  ret_arr[1] = (sen_val.theta_y_cpoint * ctrl_val.K_xz_theta + sen_val.theta_y_dot_cpoint * ctrl_val.K_xz_theta_dot) * -1;

  return ret_arr;
}




float *Controller::computeTorque(float curr_torque_arr[])
{
  static float* ret_arr = new float[3];
  //Serial.println(cos(ALPHA));
  // Serial.println(curr_torque_arr[2]);
  //compute Torque T1
  ret_arr[0] = 0.333333333 * (curr_torque_arr[2] + (2 / cos(ALPHA)) * (curr_torque_arr[0] * cos(BETA) - curr_torque_arr[1] * sin(BETA)));

  //compute Torque T2
  ret_arr[1] = 0.333333333 * (curr_torque_arr[2] + (1 / cos(ALPHA)) * (sin(BETA) * (-curr_torque_arr[0] * sqrt(3) + curr_torque_arr[1]) - cos(BETA) * (curr_torque_arr[0] + sqrt(3) * curr_torque_arr[1])));

  //compute Torque T3
  ret_arr[2] = 0.333333333 * (curr_torque_arr[2] + (1 / cos(ALPHA)) * (sin(BETA) * (curr_torque_arr[0] * sqrt(3) + curr_torque_arr[1]) + cos(BETA) * (-curr_torque_arr[0] + sqrt(3) * curr_torque_arr[1])));

  return ret_arr;

}

int *Controller::compute2currentunits(float real_torques_arr[]) {

  static int* ret_arr = new int[3];

  ret_arr[0] = round(K_EXP * real_torques_arr[0]);
  ret_arr[1] = round(K_EXP * real_torques_arr[1]);
  ret_arr[2] = round(K_EXP * real_torques_arr[2]);

//  ret_arr[0] = static_cast<int>(K_EXP * real_torques_arr[0]);
//  ret_arr[1] = static_cast<int>(K_EXP * real_torques_arr[1]);
//  ret_arr[2] = static_cast<int>(K_EXP * real_torques_arr[2]);

  return ret_arr;

}


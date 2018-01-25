
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

  for(int i = 0;i<3;i++){
      sen_val.psi_actual_[i] = 0.0;
      sen_val.psi_last_[i]   = 0.0;
      sen_val.phi_actual_[i] = 0.0;
      sen_val.phi_last_[i]   = 0.0;
      sen_val.dphi_[i]       = 0.0;
  }
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
  if(USE_CUSTOM_OFFSET){
  offset_x = custom_offset_x;
  offset_y = custom_offset_y;
  }
  else{
  offset_x = storage_imux / samples;
  offset_y = storage_imuy / samples +0.8; // TODO: Additional offset angle from center of gravity to imu
  }

  Serial.print("IMU INIT 100% finished with offset: "); Serial.print(offset_x); Serial.print(" y: "); Serial.println(offset_y);
  return true;
}


void Controller::imu_Filter(cIMU sensor)
{
  // 1. Init step do this only once:
  sensor.update();
  if (!init_imu_filter_)
  {
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

  storage_x = last_storage_x - ( last_storage_x - sensor.rpy[1] ) * FILTER_FAK;
  storage_y = last_storage_y - ( last_storage_y - sensor.rpy[0] ) * FILTER_FAK;
  storage_z = last_storage_z - ( last_storage_z - sensor.rpy[2] ) * FILTER_FAK;
  last_storage_x = storage_x;
  last_storage_y = storage_y;
  last_storage_z = storage_z;

  storage_x_dot = last_storage_x_dot - ( last_storage_x_dot - sensor.gyroData[1] ) * FILTER_FAK;
  storage_y_dot = last_storage_y_dot - ( last_storage_y_dot - sensor.gyroData[0] ) * FILTER_FAK;
  storage_z_dot = last_storage_z_dot - ( last_storage_z_dot - sensor.gyroData[2] ) * FILTER_FAK;
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

  // Store the values if the filter is not used as well:
  //Theta y,x,z in radiands
  sen_val.theta_x_cpoint_ohne_Filter = convert2radiand(sensor.rpy[1] - offset_x);
  sen_val.theta_y_cpoint_ohne_Filter = convert2radiand(sensor.rpy[0] - offset_y);
  sen_val.theta_z_cpoint_ohne_Filter = convert2radiand(sensor.rpy[2]);

  // Theta_dot y,x,z
  sen_val.theta_x_dot_cpoint_ohne_Filter = convert2radiand(sensor.gyroData[1] * gRes);
  sen_val.theta_y_dot_cpoint_ohne_Filter = convert2radiand(sensor.gyroData[0] * gRes);
  sen_val.theta_z_dot_cpoint_ohne_Filter = convert2radiand(sensor.gyroData[2] * gRes);
}



void Controller::readIMU(cIMU sensor, BallbotMotorDriver driver)
{
  float time_start = millis();

  //dass motor nicht gleich losfährt wegen anfangs IMU offset.
  if ( (abs(sensor.rpy[1] - (offset_x)) < 5 && abs(sensor.rpy[0] - (offset_y)) < 5) || init_once_ || USE_CUSTOM_OFFSET)
  {
    imu_Filter(sensor);
    init_once_ = true;

    // Read out wheel states (current[units], velocity, position[absolut])
    // CAUTION at least the effort value is only given positive or if negative as 65526
    int32_t current_effort_RAW[3]   =  {0.0, 0.0, 0.0}; // this is the present current
    int32_t current_velocity_RAW[3] =  {0.0, 0.0, 0.0};
    int32_t current_position_RAW[3] =  {0.0, 0.0, 0.0};

    driver.readWheelStates(current_effort_RAW, current_velocity_RAW, current_position_RAW); // this step costs 3ms!!!
    //calc_odometry(current_velocity_RAW);
    
    // convert negative values of current_effort_RAW:
    if (current_effort_RAW[0] > 10000) {
      current_effort_RAW[0] = current_effort_RAW[0] - 65536;
    }
    if (current_effort_RAW[1] > 10000) {
      current_effort_RAW[1] = current_effort_RAW[1] - 65536;
    }
    if (current_effort_RAW[2] > 10000) {
      current_effort_RAW[2] = current_effort_RAW[2] - 65536;
    }

    

    // Execute Controller
    xyz_2D_controller(driver); // lasts 3ms the write wheel states lasts that long!

    float time_end = millis();
    float time_duration = time_end - time_start;
    float tmp1 = 1;
    float tmp2 = 1;
    float tmp3 = 1;
    if(!PRINT_TORQUES)
    {
      tmp1 = K_M1; 
      tmp2 = K_M2; 
      tmp3 = K_M3; 
    }
    
    Serial.print(time_end / 1000);                                            Serial.print("\t"); // sec
    Serial.print(sen_val.theta_x_cpoint_ohne_Filter * 180 / 3.14159);         Serial.print("\t"); // °
    Serial.print(sen_val.theta_x_dot_cpoint_ohne_Filter * 180 / 3.14159);     Serial.print("\t"); // °/sec
    Serial.print(sen_val.theta_y_cpoint_ohne_Filter * 180 / 3.14159);         Serial.print("\t");
    Serial.print(sen_val.theta_y_dot_cpoint_ohne_Filter * 180 / 3.14159);     Serial.print("\t");

    Serial.print(sen_val.theta_x_cpoint * 180 / 3.14159);                     Serial.print("\t"); // °
    Serial.print(sen_val.theta_x_dot_cpoint * 180 / 3.14159);                 Serial.print("\t"); // °/sec
    Serial.print(sen_val.theta_y_cpoint * 180 / 3.14159);                     Serial.print("\t");
    Serial.print(sen_val.theta_y_dot_cpoint * 180 / 3.14159);                 Serial.print("\t");

    Serial.print(ctrl_val.T1*tmp1);                                         Serial.print("\t"); // effort in units drauf //9
    Serial.print(ctrl_val.T2*tmp2);                                         Serial.print("\t"); // effort in units drauf
    Serial.print(ctrl_val.T3*tmp3);                                         Serial.print("\t"); // effort in units drauf
    Serial.print(ctrl_val.T1_ohne_Filter*tmp1);                             Serial.print("\t"); // effort in units drauf
    Serial.print(ctrl_val.T2_ohne_Filter*tmp2);                             Serial.print("\t"); // effort in units drauf
    Serial.print(ctrl_val.T3_ohne_Filter*tmp3);                             Serial.print("\t"); // effort in units drauf

    // gemessener Wert:
    Serial.print(current_effort_RAW[0]);                              Serial.print("\t"); // effort in units gemessen!
    Serial.print(current_effort_RAW[1]);                              Serial.print("\t"); // effort in units gemessen!
    Serial.print(current_effort_RAW[2]);                              Serial.print("\t"); // effort in units gemessen!

    Serial.print(time_duration);

    Serial.print("\n");
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

void Controller::xyz_2D_controller(BallbotMotorDriver driver)
{
  //Compute Tx,Ty
  static float* virtual_torques = new float[2];
  static float* virtual_torques_ohne_Filter = new float[2];
  static float* real_torques = new float[2];
  static float* real_torques_ohne_Filter = new float[2];

  static float* torques_on_motors = new float[2];

  // Torque in the yz Planar --> T_x
  virtual_torques[0] = (sen_val.phi_actual_[0] * K1_X + sen_val.theta_x_cpoint * K2_X + sen_val.dphi_[0] * K3_X + sen_val.theta_x_dot_cpoint * K4_X ) * -1;

  // Torque in the xz Planar --> T_y
  virtual_torques[1] = (sen_val.phi_actual_[1] * K1_Y + sen_val.theta_y_cpoint * K2_Y + sen_val.dphi_[1] * K3_Y + sen_val.theta_y_dot_cpoint * K4_Y ) * -1;

  virtual_torques[2] = 0.0; //(sen_val.theta_z_cpoint *  1.0 + sen_val.theta_z_dot_cpoint * 1.7055) * -1;

  real_torques[0] = 0.333333333 * (virtual_torques[2] + (2 / COS_ALPHA) * (virtual_torques[0] * COS_BETA - virtual_torques[1] * SIN_BETA));
  real_torques[1] = 0.333333333 * (virtual_torques[2] + (1 / COS_ALPHA) * (SIN_BETA * (-virtual_torques[0] * SQRT3 + virtual_torques[1]) - COS_BETA * (virtual_torques[0] + SQRT3 * virtual_torques[1])));
  real_torques[2] = 0.333333333 * (virtual_torques[2] + (1 / COS_ALPHA) * (SIN_BETA * (virtual_torques[0] * SQRT3 + virtual_torques[1]) + COS_BETA * (-virtual_torques[0] + SQRT3 * virtual_torques[1])));
  ctrl_val.T1 = real_torques[0];
  ctrl_val.T2 = real_torques[1];
  ctrl_val.T3 = real_torques[2];

  // Berechne Werte ohne Filter:
  virtual_torques_ohne_Filter[0] = (sen_val.phi_actual_[0] * K1_X + sen_val.theta_x_cpoint * K2_X + sen_val.dphi_[0] * K3_X + sen_val.theta_x_dot_cpoint * K4_X ) * -1;
  virtual_torques_ohne_Filter[1] = (sen_val.phi_actual_[1] * K1_Y + sen_val.theta_y_cpoint * K2_Y + sen_val.dphi_[1] * K3_Y + sen_val.theta_y_dot_cpoint * K4_Y ) * -1;
  virtual_torques_ohne_Filter[2] = 0.0; //(sen_val.theta_z_cpoint *  1.0 + sen_val.theta_z_dot_cpoint * 1.7055) * -1;

  real_torques_ohne_Filter[0] = 0.333333333 * (virtual_torques_ohne_Filter[2] + (2 / COS_ALPHA) * (virtual_torques_ohne_Filter[0] * COS_BETA - virtual_torques_ohne_Filter[1] * SIN_BETA));
  real_torques_ohne_Filter[1] = 0.333333333 * (virtual_torques_ohne_Filter[2] + (1 / COS_ALPHA) * (SIN_BETA * (-virtual_torques_ohne_Filter[0] * SQRT3 + virtual_torques_ohne_Filter[1]) - COS_BETA * (virtual_torques_ohne_Filter[0] + SQRT3 * virtual_torques_ohne_Filter[1])));
  real_torques_ohne_Filter[2] = 0.333333333 * (virtual_torques_ohne_Filter[2] + (1 / COS_ALPHA) * (SIN_BETA * (virtual_torques_ohne_Filter[0] * SQRT3 + virtual_torques_ohne_Filter[1]) + COS_BETA * (-virtual_torques_ohne_Filter[0] + SQRT3 * virtual_torques_ohne_Filter[1])));
  ctrl_val.T1_ohne_Filter = real_torques_ohne_Filter[0];
  ctrl_val.T2_ohne_Filter = real_torques_ohne_Filter[1];
  ctrl_val.T3_ohne_Filter = real_torques_ohne_Filter[2];

  if (USE_FILTER)
  {
    torques_on_motors = real_torques;
  }
  else
  {
    torques_on_motors = real_torques_ohne_Filter;
  }
  // filter real torques right here
  // real_torques = torques_filter(real_torques, true);

  //Convert real Torques into Current
  int* curr_unit_arr = compute2currentunits(torques_on_motors);
  int torque_offset = 10; // friction
  for (int i = 0; i < 3; i++)
  {
    if (i==1) {
      if (curr_unit_arr[i] > 0)
      {
        curr_unit_arr[i] += 18;
      }
      else {
      curr_unit_arr[i] -= 18;
      }
    }
    else if (i==2) {
      if (curr_unit_arr[i] > 0)
      {
        curr_unit_arr[i] += 25;
      }
      else {
      curr_unit_arr[i] -= 25;
      }
    }
    else if (i==3) {
      if (curr_unit_arr[i] > 0)
      {
        curr_unit_arr[i] += 23;
      }
      else {
      curr_unit_arr[i] -= 23;
      }
    }
    
  }

  //Load to motors
  driver.writeServoConfig(DXM_1_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[0]);
  driver.writeServoConfig(DXM_2_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[1]);
  driver.writeServoConfig(DXM_3_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[2]);
}

void Controller::calc_odometry(int32_t velocity_RAW[])
{
    // 0. Convert the velocity from units in rad per second
    // 1 Unit = 0.229[RPM]
    float velocity_rad_s[] = {0.0, 0.0, 0.0};
    float dpsi[] = {0.0, 0.0, 0.0};
    for(int i = 0; i<3; i++)
    {
      velocity_rad_s[i]= velocity_RAW[i] * 0.229 * (2*PI)/60;
    }
    Serial.print("velocity_rad_s_0:  "); Serial.println(velocity_rad_s[0]);
    
    // 1. Convert dpsi_1,2,3 -> dpsi_x,y,z
    // 1.A dpsix = dpsi1 / cos (alpha) P.71
    // 1.B dpsiy = dpsi3 * 2 / (sqrt(3) *cos(alpha))
    // 1.C dpsiz = dpsi1/sin(alpha)
    // diese Formeln sind nicht richtig! hängt noch von Beta ab und Formeln müssen evtl. summiert werden.
    // 
    dpsi[0]= velocity_rad_s[0]/(cos(ALPHA));
    dpsi[1]= velocity_rad_s[2]/(cos(ALPHA)*SQRT3);
    dpsi[2]= velocity_rad_s[1]/(sin(ALPHA));

    Serial.print("dpsi_0:  "); Serial.println(dpsi[0]);

    // 2. Use dpsi_x,y,z -> psi_x,y,z (INTEGRATE)
    // psi_k = psi_k-1 + dpsi * sample_time
    // Integration like ETHZ: see: https://en.wikipedia.org/wiki/Trapezoidal_rule
    sen_val.psi_actual_[0] = sen_val.psi_last_[0] + SAMPL_TIME*pow(10,-6)* dpsi[0];
    sen_val.psi_actual_[1] = sen_val.psi_last_[1] + SAMPL_TIME*pow(10,-6)* dpsi[1];
    sen_val.psi_actual_[2] = sen_val.psi_last_[2] + SAMPL_TIME*pow(10,-6)* dpsi[2];

    Serial.print("sen_val.psi_actual_[0]:  "); Serial.println(sen_val.psi_actual_[0]);
    Serial.print("sen_val.psi_last_[0]:  "); Serial.println(sen_val.psi_last_[0]);

    // 3. psi_x,y,z -> dphi_x,y,z P. 8
    sen_val.dphi_[0]=(dpsi[0]+sen_val.theta_x_dot_cpoint_ohne_Filter)*RW/RK+sen_val.theta_x_dot_cpoint_ohne_Filter;
    sen_val.dphi_[1]=(dpsi[1]+sen_val.theta_y_dot_cpoint_ohne_Filter)*RW/RK+sen_val.theta_y_dot_cpoint_ohne_Filter;
    sen_val.dphi_[2]=(dpsi[2]/sin(ALPHA))*RW/RK+sen_val.theta_z_dot_cpoint_ohne_Filter;

    // 4. dphi_x,y,z -> phi_x,y,z (integrate it!)                  
    sen_val.phi_actual_[0] = sen_val.phi_last_[0]+SAMPL_TIME*pow(10,-6)*sen_val.dphi_[0];
    sen_val.phi_actual_[1] = sen_val.phi_last_[0]+SAMPL_TIME*pow(10,-6)*sen_val.dphi_[1];
    sen_val.phi_actual_[2] = sen_val.phi_last_[0]+SAMPL_TIME*pow(10,-6)*sen_val.dphi_[2];

    Serial.print("sen_val.dphi_[0]:  "); Serial.println(sen_val.dphi_[0]);    
    Serial.print("sen_val.phi_actual_[0]:  "); Serial.println(sen_val.phi_actual_[0]);
    Serial.print("sen_val.phi_last_[0]:  ");   Serial.println(sen_val.phi_last_[0]);

    
    sen_val.psi_last_ = sen_val.psi_actual_;
    sen_val.phi_last_ = sen_val.phi_actual_;
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

float Controller::convert2radiand(float val_deg)
{
  return (val_deg * PI) / 180;
}

int *Controller::compute2currentunits(float real_torques_arr[]) {

  static int* ret_arr = new int[3];

  ret_arr[0] = round(K_M1 * real_torques_arr[0]);
  ret_arr[1] = round(K_M2 * real_torques_arr[1]);
  ret_arr[2] = round(K_M3 * real_torques_arr[2]);

  //  ret_arr[0] = static_cast<int>(K_EXP * real_torques_arr[0]);
  //  ret_arr[1] = static_cast<int>(K_EXP * real_torques_arr[1]);
  //  ret_arr[2] = static_cast<int>(K_EXP * real_torques_arr[2]);

  return ret_arr;

}


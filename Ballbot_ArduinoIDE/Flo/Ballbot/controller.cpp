#include "controller.h"

extern BallbotMotorDriver motor_driver;
extern cIMU imu;


Controller::Controller()
{
  
}

Controller::~Controller()
{
  Serial.println("Destruktor IMU");
}


bool Controller::init(void)
{
  sen_val = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  double theta = 26.0564;
  double phi = 5.3985;
  ctrl_val = {-3.162,theta*FAKT,-0.5533,phi*FAKT,-3.162,theta*FAKT,-0.5533,phi*FAKT,10,SAMPLE_TIME/2,10*T1,0,0,0,0}; 
  gRes=2000.0/32768.0;

  return true;
}

bool Controller::imu_init(cIMU sensor)
{
  float storage_imux = 0.0;
  float storage_imuy = 0.0;

  
  static uint32_t tTime=0;
  int counter = 0; 


  Serial.println("take values for > 60s");
  
  while(counter < 1000){
    
    if(millis()-tTime>60 && abs(sensor.rpy[0])<4 && abs(sensor.rpy[1])<4 ){
      tTime = millis();
      sensor.update();
      storage_imux += sensor.rpy[1];
      storage_imuy += sensor.rpy[0];  
      counter++;
    }
  }
  offset_x = storage_imux/1000; 
  offset_y = storage_imuy/1000; 

  Serial.print(offset_x);
  Serial.print("\t");
  Serial.println(offset_y);
  return true;
}

void Controller::readIMU(cIMU sensor, BallbotMotorDriver driver)
{
  float time_start = millis();
  //Theta y,x,z in radiands
  
  sen_val.theta_x_cpoint = convert2radiand(sensor.rpy[1]-(offset_x));
  sen_val.theta_y_cpoint = convert2radiand(sensor.rpy[0]-(offset_y));
  sen_val.theta_z_cpoint = convert2radiand(sensor.rpy[2]);
  float current_theta_arr[3] = {sen_val.theta_x_cpoint, sen_val.theta_y_cpoint, sen_val.theta_z_cpoint};
//  sen_val.theta_y_cpoint = current_theta_arr[1];
//  sen_val.theta_x_cpoint = current_theta_arr[0];
//  sen_val.theta_z_cpoint = current_theta_arr[2];

  // Theta_dot y,x,z 
  sen_val.theta_y_dot_cpoint = convert2radiand(sensor.gyroData[0]*gRes);
  sen_val.theta_x_dot_cpoint = convert2radiand(sensor.gyroData[1]*gRes);
  sen_val.theta_z_dot_cpoint = convert2radiand(sensor.gyroData[2]*gRes);

  //Psi 1,2,3
  //Reat out the states of each Wheel.
  //The states are the current effort, velocity and position. 
  //But not modified

  if( (abs(sensor.rpy[1]-(offset_x)) <1.1 && abs(sensor.rpy[0]-(offset_y)) < 1.1) || init_once_)
  {
  //TODO:
  // CAUTION at least the effort value is only given positive or if negative as 65526
   init_once_ = true;
  int32_t current_effort_RAW[3]   =  {0.0,   0.0, 0.0}; // this is the present current
  int32_t current_velocity_RAW[3] =  {0.0, 0.0, 0.0};
  int32_t current_position_RAW[3] =  {0.0, 0.0, 0.0};
  
  driver.readWheelStates(current_effort_RAW, current_velocity_RAW, current_position_RAW);

 
  //in degree
  float current_position[3];
  for (int i=0; i<3; i++){
    current_position[i]=current_position_RAW[i]*0.088; // warum *0.088
  }

  // convert negative values of current_effort_RAW:
  if(current_effort_RAW[0]>10000)
  {
    current_effort_RAW[0]=current_effort_RAW[0]-65536;
  }

  //in radiand
  float psi_motor_1 = convert2radiand(current_position[0]); 
  float psi_motor_2 = convert2radiand(current_position[1]);
  float psi_motor_3 = convert2radiand(current_position[2]);
 
 
//rounds per second
  float current_velocity[3]; 
  for (int i=0; i<3; i++){
    current_velocity[i] = (current_velocity_RAW[i]*0.229)/60;
  }
 float psi_dot_motor_1 = current_velocity[0];
 float psi_dot_motor_2 = current_velocity[1];
 float psi_dot_motor_3 = current_velocity[2];

//Computation of angle velocity Psi_dot in the Planar Model
 float* current_psi_dot_arr = computePsiDot(current_velocity);
 sen_val.psi_x_dot_cpoint = current_psi_dot_arr[0];
 sen_val.psi_y_dot_cpoint = current_psi_dot_arr[1]; 
 sen_val.psi_z_dot_cpoint = current_psi_dot_arr[2];

 //Computation of angle Psi in the Planar Model
 float* current_psi_arr = computePsi_new2(current_position);
 sen_val.psi_x_cpoint = current_psi_arr[0];
 sen_val.psi_y_cpoint = current_psi_arr[1];
 sen_val.psi_z_cpoint = current_psi_arr[2]; 

 
  //Computation of angle velocity Phi_Dot in the Planar Model
  float* current_phi_dot_arr = computePhiDot(current_velocity);
  sen_val.phi_x_dot_cpoint = current_phi_dot_arr[0];
  sen_val.phi_y_dot_cpoint = current_phi_dot_arr[1];
  sen_val.phi_z_dot_cpoint = current_phi_dot_arr[2];

  //Computation of angle Phi in the Planar Model
  float* current_phi_arr = computePhi(current_psi_arr, current_theta_arr);
  sen_val.phi_x_dot_cpoint = current_phi_arr[0];
  sen_val.phi_y_dot_cpoint = current_phi_arr[1];
  sen_val.phi_z_dot_cpoint = current_phi_arr[2];


  //Execute Controller
  //Compute Tx,Ty,Tz
  float* curr_torque_arr = executeController2();
  
  //Execute Torque Conversion
  //Compute T1,T2,T3
  float* real_torques_arr = computeTorque(curr_torque_arr); 

  //Convert real Torques into Current
  int16_t* curr_unit_arr = compute2currentunits(real_torques_arr);

  //TODO: caution:
  //curr_unit_arr[0] = 20.0;

  //Load to motors
  driver.writeServoConfig(DXM_1_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[0]);
  driver.writeServoConfig(DXM_2_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[1]); 
  driver.writeServoConfig(DXM_3_ID, 2 , ADDR_X_GOAL_EFFORT , curr_unit_arr[2]);  


  float time_end=millis(); 

  float time_duration= time_end - time_start; 
  
  #ifdef PRINT_Values
    Serial.print(time_end/1000);                              Serial.print("\t"); // sec
    Serial.print(sen_val.theta_x_cpoint*180/3.14159);         Serial.print("\t"); // °
    Serial.print(sen_val.theta_x_dot_cpoint*180/3.14159);     Serial.print("\t"); // °/sec
    Serial.print(sen_val.theta_y_cpoint*180/3.14159);         Serial.print("\t");
    Serial.print(sen_val.theta_y_dot_cpoint*180/3.14159);     Serial.print("\t");
    
    //Serial.print(current_effort_RAW[0]);                      Serial.print("\t"); // quite high value! if negative?!
    Serial.print(real_torques_arr[0]);                        Serial.print("\t"); // real_torques_arr*11.11 = curr_unit_arr
    Serial.print(curr_unit_arr[0]);                           Serial.print("\t");

    Serial.print(real_torques_arr[1]);                        Serial.print("\t");
    Serial.print(curr_unit_arr[1]);                           Serial.print("\t");
    
    Serial.print(real_torques_arr[2]);                        Serial.print("\t");
    Serial.print(curr_unit_arr[2]);                           Serial.print("\t");
    //
    //curr_unit_arr
    Serial.print("\n");
  #endif
   
  #ifdef DEBUG_SEN
    Serial.print("Time Duration"); 
    Serial.print("\t\t"); 
    Serial.print(time_duration);
    Serial.print("\n"); 

    Serial.print("Angle[rad]");
    Serial.print("\t");
    Serial.print(sen_val.theta_y_cpoint);
    Serial.print("\t");
    Serial.print(sen_val.theta_x_cpoint);
    Serial.print("\t");
    Serial.print(sen_val.theta_z_cpoint);
    Serial.print("\n");
    Serial.print("Angle[°]");
    Serial.print("\t");
    Serial.print(sen_val.theta_y_cpoint*180/3.14159);
    Serial.print("\t");
    Serial.print(sen_val.theta_x_cpoint*180/3.14159);
    Serial.print("\t");
    Serial.print(sen_val.theta_z_cpoint*180/3.14159);
    Serial.print("\n");
    Serial.print("Velocity");
    Serial.print("\t");
    Serial.print(sen_val.theta_y_dot_cpoint);
    Serial.print("\t");
    Serial.print(sen_val.theta_x_dot_cpoint);
    Serial.print("\t");
    Serial.print(sen_val.theta_z_dot_cpoint);
    Serial.print("\n");
    Serial.print("Motor Angle");
    Serial.print("\t");
    Serial.print(psi_motor_1);
    Serial.print("\t");
    Serial.print(psi_motor_2);
    Serial.print("\t");
    Serial.print(psi_motor_2);
    Serial.print("\n");
    Serial.print("Motor Veleocity");
    Serial.print("\t");
    Serial.print(psi_dot_motor_1);
    Serial.print("\t");
    Serial.print(psi_dot_motor_2);
    Serial.print("\t");
    Serial.print(psi_dot_motor_3);
    Serial.print("\n");
    Serial.print("Planar Torques");
    Serial.print("\t");
    Serial.print(curr_torque_arr[0]);
    Serial.print("\t");
    Serial.print(curr_torque_arr[1]);
    Serial.print("\t");
    Serial.print(curr_torque_arr[2]);
    Serial.print("\n");
    Serial.print("Torques");
    Serial.print("\t");
    Serial.print(real_torques_arr[0]);
    Serial.print("\t");
    Serial.print(real_torques_arr[1]);
    Serial.print("\t");
    Serial.print(real_torques_arr[2]);
    Serial.print("\n");
    Serial.print("Units");
    Serial.print("\t");
    Serial.print(curr_unit_arr[0]);
    Serial.print("\t");
    Serial.print(curr_unit_arr[1]);
    Serial.print("\t");
    Serial.print(curr_unit_arr[2]);
    Serial.print("\n");
  #endif
  }
}
float  *Controller::computePsiDot(float omega_arr[])
{
  float *ret_arr = new float[3];
  
  //Psi_dot_x
  ret_arr[0] = omega_arr[0]/cos(ALPHA);

  //Psi_dot_x
  ret_arr[1] = (omega_arr[2]*2)/(sqrt(3)*cos(ALPHA));

  //Psi_dot_z
  ret_arr[2] = (omega_arr[0])/(sin(ALPHA));

  
  return ret_arr;
}

float *Controller::computePsi(float psi_dot_arr[])
{
  float *ret_arr = new float[3];
  static float values_psi_x[2] = {0,0};
  static float values_psi_y[2] = {0,0};
  static float values_psi_z[2] = {0,0};

  //Aktuelle Werte zuweisen
  static float current_value_psi_x_dot = psi_dot_arr[0]; 
  static float current_value_psi_y_dot = psi_dot_arr[1]; 
  static float current_value_psi_z_dot = psi_dot_arr[2];
  
  // Psi_x 
  values_psi_x[0] = current_value_psi_x_dot*SAMPLE_TIME + values_psi_x[1];
  values_psi_x[1]=values_psi_x[0];
  ret_arr[0] = values_psi_x[0]; 

   // Psi_y 
  values_psi_y[0] = current_value_psi_y_dot*SAMPLE_TIME + values_psi_y[1];
  values_psi_y[1]=values_psi_y[0];
  ret_arr[1] = values_psi_y[0]; 

   // Psi_z 
  values_psi_z[0] = current_value_psi_z_dot*SAMPLE_TIME + values_psi_z[1];
  values_psi_z[1]=values_psi_z[0];
  ret_arr[2] = values_psi_z[0]; 


  return ret_arr;
  
  
}

 float *Controller::computePsi_new2(float psi_real_arr[]){

   float *ret_arr = new float[3];

   ret_arr[0] = psi_real_arr[0]/cos(ALPHA);
   ret_arr[1] = (psi_real_arr[2]*2)/(sqrt(3)*cos(ALPHA));
   ret_arr[2] = psi_real_arr[0]/sin(ALPHA);

   return ret_arr;
 }

float *Controller::computePsi_new(float psi_real_arr[]){

  float *ret_arr = new float[3];

  static float values_psi_1[2] = {0,0};
  static float values_psi_2[2] = {0,0};
  static float values_psi_3[2] = {0,0};
  
  static float values_psi_x[2] = {0,0};
  static float values_psi_y[2] = {0,0};
  static float values_psi_z[2] = {0,0};

  //aktuellen Werte einlesen
  values_psi_1[0] = psi_real_arr[0];
  values_psi_1[1] = psi_real_arr[1];
  values_psi_1[2] = psi_real_arr[2];

  values_psi_x[0] = values_psi_x[1] + ((values_psi_1[0]-values_psi_1[1])/cos(ALPHA));
  
  values_psi_x[1]=values_psi_x[0];
  values_psi_1[1]=values_psi_1[0]; 
  ret_arr[0] = values_psi_x[0]; 

  values_psi_y[0] = values_psi_y[1] + ((values_psi_3[0]-values_psi_3[1])*2/(sqrt(3)*cos(ALPHA)));
  
  values_psi_y[1]=values_psi_y[0];
  values_psi_3[1]=values_psi_3[0]; 
  ret_arr[1] = values_psi_y[0];

   
  values_psi_z[0] = values_psi_z[1] + ((values_psi_2[0]-values_psi_3[1])/sin(ALPHA));
  values_psi_z[1]=values_psi_z[0];
  values_psi_2[1]=values_psi_2[0]; 
  ret_arr[2] = values_psi_z[0];

  return ret_arr;
  
}

float *Controller::computePhiDot(float psi_dot_arr[])
{
  float *ret_arr = new float[3];

  //Phi_dot_x
  ret_arr[0] = sen_val.theta_x_dot_cpoint + ((psi_dot_arr[0]+sen_val.theta_x_dot_cpoint)*(RW/RK));

  //Phi_dot_y
  ret_arr[1] = sen_val.theta_y_dot_cpoint + ((psi_dot_arr[1]+sen_val.theta_y_dot_cpoint)*(RW/RK));

  //Phi_dot_z
  ret_arr[2] = sen_val.theta_z_dot_cpoint*((RW)/(RK*sin(ALPHA))) + sen_val.theta_z_dot_cpoint;

  return ret_arr;
}

float *Controller::computePhi(float psi_arr[], float theta_arr[])
{
  float *ret_arr = new float[3]; 
  
  static float values_psi_x[2] = {0,0};
  static float values_psi_y[2] = {0,0};
  static float values_psi_z[2] = {0,0};

  static float values_theta_x[2] = {0,0};
  static float values_theta_y[2] = {0,0};
  static float values_theta_z[2] = {0,0};
  
  static float values_phi_x[2] = {0,0};
  static float values_phi_y[2] = {0,0};
  static float values_phi_z[2] = {0,0};

  //aktuelle Werte Psi_x_Werte zuweisen
  values_psi_x[0] = psi_arr[0];
  values_psi_y[0] = psi_arr[1];
  values_psi_z[0] = psi_arr[2];

  values_theta_x[0] = theta_arr[0];
  values_theta_y[0] = theta_arr[1];
  values_theta_z[0] = theta_arr[2];

  //Computation of Phi_x
  values_phi_x[0] = (values_psi_x[0] - values_psi_x[1] - values_theta_x[1] + values_theta_x[0]) * (RW/RK) + values_theta_x[0] - values_theta_x[1] + values_phi_x[1];
  values_phi_y[0] = (values_psi_y[0] - values_psi_y[1] + values_theta_y[1] - values_theta_y[0]) * (RW/RK) + values_theta_y[0] - values_theta_y[1] + values_phi_y[1];
  values_phi_z[0] = (values_psi_x[0] - values_psi_x[1])*((RW)/(RK*sin(ALPHA))) + values_theta_x[0] - values_theta_x[1] + values_phi_x[1];

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
  float temp_var = (val_deg * 2*PI)/360;
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
  static float e[3] = {0,0,0};
  static float u[3] = {0,0,0};

  //current value of difference e
  e[0] = sen_val.theta_z_cpoint; 

  u[0] = B2*e[0]+ B1*e[1] + B0*e[2] + u[2]; 

  e[2] = e[1]; 
  e[1] = e[0]; 

  u[2] = u[1]; 
  u[1] = u[0]; 

  ret_arr[2] = u[0]; 
  

  //shift values
  e[1] = e[0]; 
  
  return ret_arr;
}

float *Controller::executeController2()
{
  static float* ret_arr = new float[3];
  
  //Torque in the yz Planar --> T_x
  ret_arr[0] = (sen_val.theta_x_cpoint * ctrl_val.K_yz_theta +  sen_val.theta_x_dot_cpoint * ctrl_val.K_yz_theta_dot) * -1;

  // Torque in the xz Planar --> T_y
  ret_arr[1] = (sen_val.theta_y_cpoint * ctrl_val.K_xz_theta + sen_val.theta_y_dot_cpoint * ctrl_val.K_xz_theta_dot) * -1;

  // Torque in the xz Planar --> T_z
  static float e[3] = {0,0,0};
  static float u[3] = {0,0,0};

  //current value of difference e
  e[0] = sen_val.theta_z_cpoint; 

  u[0] = B2*e[0]+ B1*e[1] + B0*e[2] + u[2]; 

  e[2] = e[1]; 
  e[1] = e[0]; 

  u[2] = u[1]; 
  u[1] = u[0]; 

  ret_arr[2] = u[0]; 
  
 
  return ret_arr;
}




float *Controller::computeTorque(float curr_torque_arr[])
{
  static float* ret_arr = new float[3];
  //Serial.println(cos(ALPHA));
 // Serial.println(curr_torque_arr[2]);
  //compute Torque T1
  ret_arr[0] = 0.333333333*(curr_torque_arr[2]+(2/cos(ALPHA))*(curr_torque_arr[0]*cos(BETA)-curr_torque_arr[1]*sin(BETA))); 

  //compute Torque T2
  ret_arr[1] = 0.333333333*(curr_torque_arr[2]+(1/cos(ALPHA))*(sin(BETA)*(-curr_torque_arr[0]*sqrt(3)+curr_torque_arr[1])-cos(BETA)*(curr_torque_arr[0]+sqrt(3)*curr_torque_arr[1])));

  //compute Torque T3
  ret_arr[2] = 0.333333333*(curr_torque_arr[2]+(1/cos(ALPHA))*(sin(BETA)*(curr_torque_arr[0]*sqrt(3)+curr_torque_arr[1])+cos(BETA)*(-curr_torque_arr[0]+sqrt(3)*curr_torque_arr[1])));

  return ret_arr;
  
}

int16_t *Controller::compute2currentunits(float real_torques_arr[]){

  static int16_t* ret_arr = new int16_t[3]; 

  ret_arr[0] = (1/0.0226) * real_torques_arr[0]; 
  ret_arr[1] = (1/0.0226) * real_torques_arr[1]; 
  ret_arr[2] = (1/0.0226) * real_torques_arr[2];

//  ret_arr[0] = round(K_EXP * real_torques_arr[0]); 
//  ret_arr[1] = round(K_EXP * real_torques_arr[1]); 
//  ret_arr[2] = round(K_EXP * real_torques_arr[2]); 

  return ret_arr; 
  
}


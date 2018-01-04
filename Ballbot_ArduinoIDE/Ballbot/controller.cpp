#include "controller.h"

sensor_values sen_val = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

float gRes = 2000.0/32768.0;


void readIMU(cIMU device)
{
  //Theta y,x,z in radiands
  sen_val.theta_y_cpoint = convert2radiand(device.rpy[0]);
  sen_val.theta_x_cpoint = convert2radiand(device.rpy[1]);
  sen_val.theta_z_cpoint = convert2radiand(device.rpy[2]);

  // Theta_dot y,x,z 
  sen_val.theta_y_dot_cpoint = convert2radiand(device.gyroData[0]*gRes);
  sen_val.theta_x_dot_cpoint = convert2radiand(device.gyroData[1]*gRes);
  sen_val.theta_z_dot_cpoint = convert2radiand(device.gyroData[2]*gRes);

  //Position of Motor
    

  //Print Angel
 
  #ifdef DEBUG_ANGLE
    Serial.print(device.angle[0]);
    Serial.print("\t");
    Serial.print(device.angle[1]);
    Serial.print("\t");
    Serial.print(device.angle[2]);
    Serial.print("\n"); 
  #endif

  #ifdef DEBUG_VEL
    Serial.print(device.gy);
    Serial.print("\t");
    Serial.print(device.gx);
    Serial.print("\t");
    Serial.print(device.gz);
    Serial.print("\n"); 
  #endif
};

float convert2radiand(float val_deg)
{
  float temp_var = (val_deg * 2*PI)/360;
  return temp_var;
};



//Controller::Controller()
//{
  
//}

//Controller::~Controller()
//{
  
//}


//void Controller::readIMU(cIMU imu)
//{
  //imu.update();

  //float temp_value_rnd = imu.rpy[0]; 

  
  
  //Serial.print(imu.rpy[0]);
  //Serial.print("\n");
//};


//float Controller::convert2radian(float x) 
//{
 //return 0;
//};


//float Controller::getPhiValue(float theta, float psi)
//{  
  //return 0;
//};

#include "controller.h"


Controller::Controller()
{
  
}

Controller::~Controller()
{
  
}


void Controller::init()
{
  sen_val = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  ctrl_val = {-3.162,24.8947,-0.5533,7.1600,-3.162,24.8947,-0.5533,7.1600,2.9800, 1.000, 0.100, 0,0,0,0}; 
  gRes=2000.0/32768.0;
}

void Controller::readIMU(cIMU sensor)
{
  //Theta y,x,z in radiands
  sen_val.theta_y_cpoint = convert2radiand(sensor.rpy[0]);
  sen_val.theta_x_cpoint = convert2radiand(sensor.rpy[1]);
  sen_val.theta_z_cpoint = convert2radiand(sensor.rpy[2]);

  // Theta_dot y,x,z 
  sen_val.theta_y_dot_cpoint = convert2radiand(sensor.gyroData[0]*gRes);
  sen_val.theta_x_dot_cpoint = convert2radiand(sensor.gyroData[1]*gRes);
  sen_val.theta_z_dot_cpoint = convert2radiand(sensor.gyroData[2]*gRes);
  
  
    
  #ifdef DEBUG_ANGLE
    Serial.print(sen_val.theta_y_cpoint);
    Serial.print("\t");
    Serial.print(sen_val.theta_x_cpoint);
    Serial.print("\t");
    Serial.print(sen_val.theta_z_cpoint);
    Serial.print("\n");
  #endif

  #ifdef DEBUG_VELOCITY
    Serial.print(sen_val.theta_y_dot_cpoint);
    Serial.print("\t");
    Serial.print(sen_val.theta_x_dot_cpoint);
    Serial.print("\t");
    Serial.print(sen_val.theta_z_dot_cpoint);
    Serial.print("\n");
  #endif
  
 
}

float Controller::convert2radiand(float val_deg)
{
  float temp_var = (val_deg * 2*PI)/360;
  return temp_var;  
}


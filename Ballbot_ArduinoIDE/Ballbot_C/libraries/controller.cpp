#include "controller.h"

Controller::Controller()
{
  
}

Controller::~Controller()
{
  
}


void Controller::readIMU(cIMU imu)
{
  imu.update();

  float temp_value_rnd = imu.rpy[0]; 

  
  
  Serial.print(imu.rpy[0]);
  Serial.print("\n");
};


float Controller::convert2radian(float x) 
{
 return 0;
};


float Controller::getPhiValue(float theta, float psi)
{  
  return 0;
};
  



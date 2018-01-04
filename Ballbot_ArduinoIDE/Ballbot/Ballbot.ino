#include "controller.h"

cIMU    IMU;
HardwareTimer Timer(TIMER_CH1);


void setup()
{
  Serial.begin(115200);
  IMU.begin();
  Timer.stop();
  Timer.setPeriod(100000);
  Timer.attachInterrupt(executeController);
  Timer.start();

}

void loop()
{

  IMU.update();
  
}

void executeController()
{
    readIMU(IMU);
    
}



#include "controller.h"
 

cIMU imu;
Controller controller;
HardwareTimer Timer(TIMER_CH1);


void setup()
{
  Serial.begin(115200);

  imu.begin();
  controller.init();
  
  Timer.stop();
  Timer.setPeriod(100000);
  Timer.attachInterrupt(executeController);
  Timer.start();

}

void loop()
{

  imu.update();
  
}

void executeController()
{
    controller.readIMU(imu);
    
}



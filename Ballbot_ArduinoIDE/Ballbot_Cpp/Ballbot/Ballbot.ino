#include "controller.h"
#include "ballbot_motor_driver.h"

cIMU imu;
Controller controller;
BallbotMotorDriver motor_driver;
HardwareTimer Timer(TIMER_CH1);


void setup()
{
  Serial.begin(115200);

  if(imu.begin()==IMU_OK)     Serial.println("IMU initialized");
  else                        Serial.println("Error IMU");
  
  if(controller.init())       Serial.println("Controller initialized");
  else                        Serial.println("Error Controller");
  
  if(motor_driver.init())     Serial.println("Motors initialized");
  else                        Serial.println("Error Motors");

  delay(3000);
  
  Timer.stop();
  Timer.setPeriod(SAMPL_TIME);
  Timer.attachInterrupt(executeController);
  Timer.start();

}

void loop()
{
  imu.update(); 

}

void executeController(void)
{
    controller.readIMU(imu,motor_driver);
    
}



#include "controller.h"
#include "ballbot_motor_driver.h"

cIMU imu;
Controller controller;
BallbotMotorDriver motor_driver;
HardwareTimer Timer(TIMER_CH1);


void setup()
{
  Serial.begin(115200);
  while (!Serial) {
  }
  
  Serial.println("Start initialization");
  controller.init();
  imu.begin(); // update IMU with 200Hz = 5ms damit neuer wert != alter wert
  motor_driver.init();
  Serial.println("End initialization");

  //Change mode to current Control Mode:
  motor_driver.change_all_Modes(0);
  
  delay(1000);
  
//    int16_t angle[3];
//  float   rpy[3];
//  float   quat[4];
//  int16_t gyroData[3];
//  int16_t gyroRaw[3];
//  int16_t accData[3];
//  int16_t accRaw[3];
//  int16_t magData[3];
//  int16_t magRaw[3];


  Timer.stop();
  Timer.setPeriod(SAMPL_TIME);
  Timer.attachInterrupt(executeController);
  Timer.start();
  Serial.println("Controller started");
}

void loop()
{


}

void executeController(void)
{
    imu.update(); //0.32ms do not put this command in readIMU !
    controller.readIMU(imu, motor_driver);
}


// Notes:
// The IMU's orientation is calculated by using the acc and the angular vel (gyro data) as well as the magnetometer.
// as input of the Madgwick Library (see: https://github.com/arduino-libraries/MadgwickAHRS)
// The motors we use are the: http://support.robotis.com/en/product/actuator/dynamixel_x/xm_series/xm430-w350.htm#bookmark2
// Firmware version: 41
// We set the Drive Mode of all Motors to REVERSE (1) as the motors are turned in our case!


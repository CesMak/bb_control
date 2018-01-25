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
  //controller.test_IMU_FILTER(imu);
  controller.imu_init(imu, 100);
  motor_driver.init();
  Serial.println("End initialization");

  //motor_driver.referenzFahrt(); // required for odometry!

  //Change mode to current Control Mode:
  motor_driver.change_all_Modes(0);
  
  delay(1000);

  // Dead Time analysis: 7ms
  //controller.do_Step(motor_driver);
  //delay(10000);
  
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
    imu.update(); // do not put this command in readIMU !
    controller.readIMU(imu, motor_driver);
}


// Notes:
// The IMU's orientation is calculated by using the acc and the angular vel (gyro data) as well as the magnetometer.
// as input of the Madgwick Library (see: https://github.com/arduino-libraries/MadgwickAHRS)
// The motors we use are the: http://support.robotis.com/en/product/actuator/dynamixel_x/xm_series/xm430-w350.htm#bookmark2
// Firmware version: 41
// We set the Drive Mode of all Motors to REVERSE (1) as the motors are turned in our case!


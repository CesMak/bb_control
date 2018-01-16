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

  delay(1000);
  Serial.println("Start initialization");
  imu.begin();
  controller.imu_init(imu, 100);
  controller.init();
  motor_driver.init();
  Serial.println("End initialization");

  //motor_driver.referenzFahrt();

  //Change mode to current Control Mode:
  motor_driver.changeMode(0);
  
  delay(3000);
  
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
    imu.update(); 
    controller.readIMU(imu,motor_driver);
}


// Notes:
// The IMU's orientation is calculated by using the acc and the angular vel (gyro data) as well as the magnetometer.
// as input of the Madgwick Library (see: https://github.com/arduino-libraries/MadgwickAHRS)


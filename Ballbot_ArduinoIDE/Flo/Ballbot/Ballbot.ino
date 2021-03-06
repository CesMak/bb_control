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
    ; 
  }
  
  Serial.println("Init started");

  imu.begin();

  controller.imu_init(imu);

  Serial.println("After imu_init");

  controller.init();

  motor_driver.init();

  Serial.println("Init finished");
  
  
//  
//  while(imu.begin() != IMU_OK){
//      Serial.println("IMU not ready");
//      }
////  if(imu.begin()==IMU_OK){
////    Serial.println("IMU initialized");
////  }
////  else{
////    Serial.println("Error IMU");
////  }
//
//  // init imu:
//  // first 5 secs the imu is shity
//  while (!Serial.available());
//  imu.update();
//  
//  if(controller.imu_init(imu)){
//    Serial.println("IMU offset initialized");
//  }
//  else{
//    Serial.println("Error IMU init");
//  }
//  while (!Serial.available());
//  if(controller.init()){
//    Serial.println("Controller initialized");
//  }
//  else{
//    Serial.println("Error Controller");
//  }
//  
//  if(motor_driver.init()){
//    Serial.println("Motors initialized");
//  }
//  else{
//    Serial.println("Error Motors");
//  }

//  motor_driver.setTorque(DXM_1_ID,false);
//  motor_driver.setTorque(DXM_2_ID,false);
//  motor_driver.setTorque(DXM_3_ID,false);
//
//  motor_driver.changeMode(DXM_1_ID,3);
//  motor_driver.changeMode(DXM_2_ID,3);
//  motor_driver.changeMode(DXM_3_ID,3);
//
//  motor_driver.setTorque(DXM_1_ID,true);
//  motor_driver.setTorque(DXM_2_ID,true);
//  motor_driver.setTorque(DXM_3_ID,true);
//  
//  //ZeroPosition
//  int32_t wheel_effort_values[3] = {0,0,0};
//  int32_t wheel_velocity_values[3] = {0,0,0};
//  int32_t wheel_position_values[3] = {0,0,0};
//  int goal = 0; 
// 
//  motor_driver.writeServoConfig(DXM_1_ID, 4 , ADDR_X_GOAL_POSITION , goal);
//  do
//  { 
//    motor_driver.readWheelStates(wheel_effort_values,wheel_velocity_values,wheel_position_values);
//    //Serial.println(wheel_position_values[0]);
//  }while(abs(goal - wheel_position_values[0]) > DXL__POS_THRESHOLD);
//
//  Serial.println("M1 okay");
//  
//  motor_driver.writeServoConfig(DXM_2_ID, 4 , ADDR_X_GOAL_POSITION , goal);
//  do
//  { 
//    motor_driver.readWheelStates(wheel_effort_values,wheel_velocity_values,wheel_position_values);
//    //Serial.println(wheel_position_values[1]);
// 
//  }while(abs(goal - wheel_position_values[1]) > DXL__POS_THRESHOLD);
//
//  Serial.println("M2 okay");
//  
//  motor_driver.writeServoConfig(DXM_3_ID, 4 , ADDR_X_GOAL_POSITION , goal);
//  do
//  { 
//    motor_driver.readWheelStates(wheel_effort_values,wheel_velocity_values,wheel_position_values);
//    //Serial.println(wheel_position_values[2]);
// 
//  }while(abs(goal - wheel_position_values[2]) > DXL__POS_THRESHOLD);
//
//  Serial.println("M3 okay");

  motor_driver.setTorque(DXM_1_ID,false);
  motor_driver.setTorque(DXM_2_ID,false);
  motor_driver.setTorque(DXM_3_ID,false);

  motor_driver.changeMode(DXM_1_ID,0);
  motor_driver.changeMode(DXM_2_ID,0);
  motor_driver.changeMode(DXM_3_ID,0);

  motor_driver.setTorque(DXM_1_ID,true);
  motor_driver.setTorque(DXM_2_ID,true);
  motor_driver.setTorque(DXM_3_ID,true);
  
  delay(3000);
  
  Timer.stop();
  Timer.setPeriod(SAMPLE_TIME);
  Timer.attachInterrupt(executeController);
  Timer.start();

}

void loop()
{


}

void executeController(void)
{
    imu.update(); 
    controller.readIMU(imu,motor_driver);
}


